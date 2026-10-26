"""Real SO-101 driver: the same ROS interface as the Gazebo controllers, on top of the Feetech servo bus.

  ros2 run so101_cell so101_driver --ros-args -p port:=/dev/ttyACM0 -p calibration:=~/.../my_follower.json
  ros2 run so101_cell so101_driver --ros-args -p port:=fake      # no hardware: joints follow the goals

Provides /arm_controller/follow_joint_trajectory, /gripper_controller/follow_joint_trajectory
(control_msgs/FollowJointTrajectory) and /joint_states. Trajectories are interpolated linearly between
their points and streamed to the servos at ``rate`` Hz; the servos' own position loop does the rest.
"""
import math
import os
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_cell.feetech import MOTORS, BusError, FeetechBus, JointMap, load_calibration
from so101_cell.kinematics import ARM_JOINTS, GRIPPER_JOINT

DEFAULT_CALIBRATION = '~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json'


class FakeBus:
    """Stands in for the servos: every joint is wherever it was last told to go."""

    def __init__(self, joint_map):
        self.jm = joint_map
        self.raw = {MOTORS[j]: joint_map.to_raw(j, 0.0) for j in MOTORS}

    def sync_read_positions(self, ids):
        return {i: self.raw[i] for i in ids}

    def sync_write_goals(self, goals):
        self.raw.update(goals)

    def torque(self, ids, on):
        pass

    def configure(self, ids, gripper_id=None):
        pass

    def close(self):
        pass


def fake_calibration():
    return {name: {'id': i, 'drive_mode': 0, 'homing_offset': 0, 'range_min': 1000, 'range_max': 3095}
            for name, i in MOTORS.items()}


class SO101Driver(Node):
    def __init__(self):
        super().__init__('so101_driver')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        calib_path = os.path.expanduser(self.declare_parameter('calibration', DEFAULT_CALIBRATION).value)
        self.rate = float(self.declare_parameter('rate', 50.0).value)
        # Final joint error (rad) a trajectory may end with; the servos sag a little under gravity.
        self.goal_tolerance = float(self.declare_parameter('goal_tolerance', 0.12).value)
        self.release_on_exit = bool(self.declare_parameter('release_on_exit', True).value)

        if port == 'fake':
            self.jm = JointMap(fake_calibration())
            self.bus = FakeBus(self.jm)
        else:
            if not os.path.exists(calib_path):
                raise RuntimeError(f'no calibration at {calib_path}; run `ros2 run so101_cell so101_calibrate` first')
            self.jm = JointMap(load_calibration(calib_path))
            self.bus = FeetechBus(port)
            self.bus.torque(MOTORS.values(), False)
            self.bus.configure(MOTORS.values(), gripper_id=MOTORS[GRIPPER_JOINT])
        self.ids = [MOTORS[j] for j in MOTORS]
        self.names = list(MOTORS)
        self.bus_lock = threading.Lock()
        self.position = self._read()  # rad, by joint name
        self.setpoint = dict(self.position)  # start by holding where the arm is
        self.get_logger().info('joints at start: ' + ', '.join(f'{j} {v:+.2f}' for j, v in self.position.items()))
        self.bus.torque(self.ids, True)

        self.goal_lock = threading.Lock()
        self.active = {}  # controller -> goal id, so a new goal preempts the old one
        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        for controller, joints in (('arm_controller', ARM_JOINTS), ('gripper_controller', [GRIPPER_JOINT])):
            ActionServer(self, FollowJointTrajectory, f'/{controller}/follow_joint_trajectory',
                         execute_callback=self._executor_for(controller, joints),
                         goal_callback=self._check_goal(joints),
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.running = True
        self.loop = threading.Thread(target=self._control_loop, daemon=True)
        self.loop.start()

    # ---------- servo I/O ----------
    def _read(self):
        with self.bus_lock:
            raw = self.bus.sync_read_positions(self.ids)
        return {j: self.jm.to_rad(j, raw[MOTORS[j]]) for j in self.names}

    def _control_loop(self):
        period = 1.0 / self.rate
        errors = 0
        while self.running and rclpy.ok():
            t0 = time.monotonic()
            try:
                goals = {MOTORS[j]: self.jm.to_raw(j, v) for j, v in dict(self.setpoint).items()}
                with self.bus_lock:
                    self.bus.sync_write_goals(goals)
                self.position = self._read()
                errors = 0
            except BusError as e:
                errors += 1
                if errors in (3, 100):
                    self.get_logger().error(f'servo bus: {e}')
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.names
            msg.position = [self.position[j] for j in self.names]
            self.js_pub.publish(msg)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def shutdown(self):
        self.running = False
        self.loop.join(timeout=1.0)
        if self.release_on_exit:
            with self.bus_lock:
                try:
                    self.bus.torque(self.ids, False)
                except BusError:
                    pass
        self.bus.close()

    # ---------- trajectory execution ----------
    def _check_goal(self, joints):
        def check(goal):
            names = list(goal.trajectory.joint_names)
            if not goal.trajectory.points or set(names) - set(joints):
                self.get_logger().warn(f'rejected trajectory for joints {names}')
                return GoalResponse.REJECT
            margin = 0.02 if joints != [GRIPPER_JOINT] else 1.0  # a gripper goal past its range just squeezes
            for p in goal.trajectory.points:
                for j, v in zip(names, p.positions):
                    lo, hi = self.jm.limits(j)
                    if not lo - margin <= v <= hi + margin:
                        self.get_logger().warn(f'rejected trajectory: {j} {math.degrees(v):.0f} deg is outside its '
                                               f'calibrated range {math.degrees(lo):.0f}..{math.degrees(hi):.0f} deg '
                                               '(recalibrate, moving each joint all the way)')
                        return GoalResponse.REJECT
            return GoalResponse.ACCEPT
        return check

    def _executor_for(self, controller, joints):
        def execute(handle):
            traj = handle.request.trajectory
            names = list(traj.joint_names)
            with self.goal_lock:
                self.active[controller] = handle.goal_id.uuid.tobytes()
            # Waypoints as (t, positions) starting from the current setpoint.
            start = [self.setpoint[j] for j in names]
            pts = [(0.0, start)] + [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, list(p.positions))
                                    for p in traj.points]
            t0, k = time.monotonic(), 1
            result = FollowJointTrajectory.Result()
            while k < len(pts):
                if handle.is_cancel_requested:
                    handle.canceled()
                    return result
                with self.goal_lock:
                    if self.active.get(controller) != handle.goal_id.uuid.tobytes():
                        handle.abort()  # preempted by a newer goal
                        return result
                t = time.monotonic() - t0
                while k < len(pts) and t >= pts[k][0]:
                    k += 1
                if k < len(pts):
                    (ta, qa), (tb, qb) = pts[k - 1], pts[k]
                    s = 0.0 if tb <= ta else (t - ta) / (tb - ta)
                    q = [a + (b - a) * s for a, b in zip(qa, qb)]
                else:
                    q = pts[-1][1]
                for j, v in zip(names, q):
                    self.setpoint[j] = v
                time.sleep(1.0 / self.rate)

            final = dict(zip(names, pts[-1][1]))
            if controller == 'gripper_controller':  # closing on a cube stalls the jaw, which is fine
                time.sleep(0.3)
                handle.succeed()
                return result
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                err = max(abs(self.position[j] - v) for j, v in final.items())
                if err <= self.goal_tolerance:
                    break
                time.sleep(0.02)
            if err > self.goal_tolerance:
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f'joint error {math.degrees(err):.1f} deg after the trajectory'
                self.get_logger().warn(result.error_string)
                handle.abort()
                return result
            handle.succeed()
            return result
        return execute


def main():
    rclpy.init()
    node = SO101Driver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
