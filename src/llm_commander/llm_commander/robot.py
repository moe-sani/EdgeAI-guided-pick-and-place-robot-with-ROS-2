"""Pick-and-place skills for the SO-101: top-down IK + FollowJointTrajectory (same in sim and on the real arm)."""
import math
import threading
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from llm_commander.scene import cube_half_size
from so101_cell.kinematics import ARM_JOINTS, GRIPPER_JOINT, IKError, SO101Kinematics

LINE_STEP = 0.005  # m between waypoints of a straight move


class SkillError(Exception):
    pass


def _yaw(p):
    o = p.orientation
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y ** 2 + o.z ** 2))


def _duration(t):
    return Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))


class RobotSkills:
    def __init__(self, node, scene, destinations, get_pose, all_poses, log):
        self.node, self.scene, self.dests = node, scene, destinations
        self.get_pose, self.all_poses, self.log = get_pose, all_poses, log
        self.m = scene['motion']
        urdf = f"{get_package_share_directory('so101_cell')}/urdf/so101_arm.xacro"
        self.kin = SO101Kinematics.from_file(urdf)
        self.arm_client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip_client = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.joints, self.joints_lock = {}, threading.Lock()
        self._picked = None  # last object picked, for when the camera can't see it in the gripper
        self._goal = None  # last arm pose commanded and reached
        node.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_joints(self, msg):
        with self.joints_lock:
            self.joints.update(zip(msg.name, msg.position))

    def arm_joints(self):
        with self.joints_lock:
            if not all(j in self.joints for j in ARM_JOINTS):
                raise SkillError('No /joint_states from the robot (is the simulation or the driver running?).')
            return np.array([self.joints[j] for j in ARM_JOINTS])

    # ---------- low-level action helpers ----------
    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise SkillError('Timed out waiting for the robot action server.')
        return future.result()

    def _trajectory(self, client, joints, points, what):
        """Run ``points`` = [(time_from_start, positions), ...] on a controller."""
        if not client.wait_for_server(timeout_sec=5.0):
            raise SkillError(f'{what}: controller not available (is the simulation or the driver running?).')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        for t, q in points:
            goal.trajectory.points.append(JointTrajectoryPoint(positions=[float(v) for v in q],
                                                               time_from_start=_duration(t)))
        handle = self._wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise SkillError(f'{what}: goal rejected.')
        res = self._wait(handle.get_result_async(), points[-1][0] + 30.0).result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._goal = None
            raise SkillError(f'{what} failed: {res.error_string or res.error_code}')
        if client is self.arm_client:
            self._goal = np.array(points[-1][1], float)
            self._settle(self._goal)

    def _settle(self, q, tol=0.01, timeout=1.0):
        """Wait until /joint_states shows the arm at ``q`` (the controller may report success a little early)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and float(np.max(np.abs(self.arm_joints() - q))) > tol:
            time.sleep(0.02)

    def _start(self):
        """Where the next move starts: the last commanded pose if the arm is there (the real servos sag a
        little under gravity, which would otherwise tilt each planned line), else the measured joints."""
        q = self.arm_joints()
        if self._goal is not None and float(np.max(np.abs(q - self._goal))) < 0.15:
            return self._goal
        return q

    def move_joints(self, q, what, duration=None):
        q = np.asarray(q, float)
        # Slow down long moves (rad/s of the fastest joint), so the real arm doesn't jerk.
        t = max(duration or self.m['travel_time'], float(np.max(np.abs(q - self.arm_joints()))) / 1.0)
        self._trajectory(self.arm_client, ARM_JOINTS, [(t, q)], what)

    def move_line(self, z, what):
        """Straight vertical tool move from where the arm is to height ``z``, at fine_speed."""
        q0 = self._start()
        start = self.kin.fk(q0)[:3, 3]
        dist = abs(z - start[2])
        steps = max(2, int(math.ceil(dist / LINE_STEP)))
        try:
            qs = self.kin.line(q0, [start[0], start[1], z], steps)
        except IKError as e:
            raise SkillError(f'{what}: {e}')
        dt = dist / steps / self.m['fine_speed']
        self._trajectory(self.arm_client, ARM_JOINTS, [(dt * (i + 1), q) for i, q in enumerate(qs)], what)

    def above(self, x, y, yaw, what):
        """Joint move to the travel height over (x, y), jaws closing along ``yaw``."""
        try:
            q = self.kin.solve_down([x, y, self.m['approach_z']], yaw, self.arm_joints())
        except IKError as e:
            raise SkillError(f'{what}: ({x:.3f}, {y:.3f}) is out of the robot\'s reach ({e}).')
        self.move_joints(q, what)

    def gripper(self, value):
        self._trajectory(self.grip_client, [GRIPPER_JOINT], [(0.8, [value])], 'Gripper')

    def open_gripper(self):
        self.gripper(self.m['gripper_open'])

    def home(self):
        self.move_joints(self.m['home_joints'], 'Move home')

    # ---------- skills ----------
    def holding(self):
        """Name of the held object, inferred from the world: only a gripped cube is well above the table.

        Stateless on purpose, so it survives commander restarts, one-shot runs and scene resets. On the real
        robot the camera only reports cubes resting on the table, so a held cube is just one the camera can't
        see; this then returns None and a failed pick is caught by the cube still being on the table.
        """
        lift_z = self.scene['table_z'] + cube_half_size() + 0.02
        time.sleep(0.3)  # let fresh poses arrive after the robot stops
        for name, p in self.all_poses().items():
            if p.position.z > lift_z:
                return name
        return None

    def pick(self, obj):
        """Grasp ``obj``, lift it to the travel height and keep holding it."""
        held = self.holding()
        if held:
            raise SkillError(f'Already holding {held}; place it first.')
        m = self.m
        pose = self.get_pose(obj)
        if pose is None:
            raise SkillError(f'No pose received for {obj}; is it on the table (and in view of the camera)?')
        p = pose.position
        self.log(f'  pick {obj} at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        self.open_gripper()
        self.above(p.x, p.y, _yaw(pose), f'Move above {obj}')
        self.move_line(self.scene['table_z'] + m['grasp_z'], f'Descend to {obj}')
        self.gripper(m['gripper_close'])
        time.sleep(0.3)
        self.move_line(m['approach_z'], f'Lift {obj}')
        self._picked = obj

        # Failed if the cube is still seen resting where it was (the camera can't see it in the gripper).
        after = self.get_pose(obj, since=time.monotonic())
        if after is not None and after.position.z < self.scene['table_z'] + cube_half_size() + 0.02 \
                and math.hypot(after.position.x - p.x, after.position.y - p.y) < 0.05:
            self._picked = None
            self.open_gripper()
            raise SkillError(f'Grasp failed: {obj} did not lift with the gripper.')
        return f'Holding {obj}.'

    def _held(self):
        """The held object: lifted in the world (sim), or picked and not seen on the table since (camera)."""
        held = self.holding()
        if held is None and self._picked and self.get_pose(self._picked, since=time.monotonic()) is None:
            held = self._picked
        return held

    def place(self, dest_name):
        """Put the held object into ``dest_name``."""
        obj = self._held()
        if obj is None:
            raise SkillError('Not holding anything to place.')
        m, d = self.m, self.dests[dest_name]
        yaw = self.kin.fk(self._start())[:3, :3]
        yaw = math.atan2(yaw[1, 0], yaw[0, 0])  # keep the jaws turned as they are
        x, y = self._free_spot(d, obj)
        self.log(f'  place {obj} into {dest_name} at ({x:.3f}, {y:.3f})')
        self.above(x, y, yaw, f'Move above {dest_name}')
        self.move_line(d['release_z'], f'Lower into {dest_name}')
        self.gripper(m.get('gripper_release', m['gripper_open']))
        time.sleep(0.5)
        self.move_line(m['approach_z'], f'Retreat from {dest_name}')
        self._picked = None

        final = self.get_pose(obj, since=time.monotonic())
        if final is None:
            raise SkillError(f'{obj} is not seen anywhere after placing it in {dest_name}.')
        if not self._inside(final, d):
            f = final.position
            raise SkillError(f'{obj} did not end up inside {dest_name}; it is at ({f.x:.3f}, {f.y:.3f}, {f.z:.3f}).')
        return f'{obj} is now in {dest_name} at ({final.position.x:.3f}, {final.position.y:.3f}).'

    def pick_and_place(self, obj, dest_name):
        if self._held() != obj:
            pose = self.get_pose(obj)
            if pose is not None and self._inside(pose, self.dests[dest_name]):
                return f'{obj} is already in {dest_name}.'
            self.pick(obj)
        return self.place(dest_name)

    def _free_spot(self, d, obj):
        """Drop point in ``d`` farthest from the objects already in it (the centre while it is empty)."""
        others = [(p.position.x, p.position.y) for n, p in self.all_poses().items() if n != obj and self._inside(p, d)]
        if not others:
            return d['x'], d['y']
        # Cube centres this far from the middle still keep the cube and the open jaws off the walls.
        off_x, off_y = (max(0.0, h - 1.7 * cube_half_size()) for h in (d['half_x'], d['half_y']))
        spots = [(d['x'] + sx * off_x, d['y'] + sy * off_y) for sx in (-1, 0, 1) for sy in (-1, 0, 1)]
        return max(spots, key=lambda s: min(math.hypot(s[0] - ox, s[1] - oy) for ox, oy in others))

    def _inside(self, pose, d):
        # Resting inside the footprint, and not still hanging in the gripper above it.
        p = pose.position
        return (abs(p.x - d['x']) <= d['half_x'] and abs(p.y - d['y']) <= d['half_y']
                and p.z < d['floor_z'] + 3 * cube_half_size())
