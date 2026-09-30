"""Pick-and-place skills on top of the ros2srrc /Robmove and /Move action servers."""
import math
import threading
import time

from rclpy.action import ActionClient
from ros2srrc_data.action import Move, Robmove
from ros2srrc_data.msg import Joints

from llm_commander.scene import cube_half_size


class SkillError(Exception):
    pass


def _yaw(p):
    return math.atan2(2.0 * (p.qw * p.qz + p.qx * p.qy), 1.0 - 2.0 * (p.qy ** 2 + p.qz ** 2))


def grasp_quaternion(cube_yaw):
    """Gripper pointing down, rotated about world Z to match the cube's nearest 90-degree face."""
    psi = (cube_yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return (-math.sin(psi / 2.0), math.cos(psi / 2.0), 0.0, 0.0)


class RobotSkills:
    def __init__(self, node, scene, destinations, get_pose, log):
        self.node, self.scene, self.dests = node, scene, destinations
        self.get_pose, self.log = get_pose, log
        self.m = scene['motion']
        self.robmove_client = ActionClient(node, Robmove, '/Robmove')
        self.move_client = ActionClient(node, Move, '/Move')

    # ---------- low-level action helpers ----------
    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise SkillError('Timed out waiting for the robot action server.')
        return future.result()

    def _send(self, client, goal, what, timeout=120.0):
        if not client.wait_for_server(timeout_sec=5.0):
            raise SkillError(f'Action server for {what} is not available (is the simulation running?).')
        handle = self._wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise SkillError(f'{what}: goal rejected.')
        return self._wait(handle.get_result_async(), timeout).result

    def robmove(self, kind, x, y, z, q, speed):
        g = Robmove.Goal()
        g.type, g.speed = kind, float(speed)
        g.x, g.y, g.z = float(x), float(y), float(z)
        g.qx, g.qy, g.qz, g.qw = (float(v) for v in q)
        res = self._send(self.robmove_client, g, f'RobMove {kind}')
        if not res.success:
            raise SkillError(f'RobMove {kind} to ({x:.3f}, {y:.3f}, {z:.3f}) failed: {res.message}')

    def _move(self, goal, what):
        res = self._send(self.move_client, goal, what)
        if 'SUCCESS' not in res.result.upper():
            raise SkillError(f'{what} failed: {res.result}')

    def gripper(self, value):
        g = Move.Goal()
        g.action, g.speed, g.moveg = 'MoveG', 1.0, float(value)
        self._move(g, 'Gripper')

    def home(self):
        g = Move.Goal()
        g.action, g.speed = 'MoveJ', self.m['travel_speed']
        j = self.m['home_joints_deg']
        g.movej = Joints(joint1=j[0], joint2=j[1], joint3=j[2], joint4=j[3], joint5=j[4], joint6=j[5])
        self._move(g, 'MoveJ home')

    # ---------- skills ----------
    def pick_and_place(self, obj, dest_name):
        m, d = self.m, self.dests[dest_name]
        pose = self.get_pose(obj)
        if pose is None:
            raise SkillError(f'No pose received for {obj}; is it spawned?')
        if self._inside(pose, d):
            return f'{obj} is already in {dest_name}.'

        q = grasp_quaternion(_yaw(pose))
        grasp_z = pose.z + m['grasp_z_offset']
        self.log(f'  pick {obj} at ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})')
        self.gripper(0.0)
        self.robmove('PTP', pose.x, pose.y, m['approach_z'], q, m['travel_speed'])
        self.robmove('LIN', pose.x, pose.y, grasp_z, q, m['fine_speed'])
        self.gripper(m['gripper_close'])
        time.sleep(0.5)
        self.robmove('LIN', pose.x, pose.y, m['approach_z'], q, m['fine_speed'])

        lifted = self.get_pose(obj)
        if lifted is None or lifted.z < pose.z + 0.05:
            self.gripper(0.0)
            raise SkillError(f'Grasp failed: {obj} did not lift with the gripper.')

        release_z = d['release_z_center'] + m['grasp_z_offset']
        self.log(f'  place into {dest_name} at ({d["x"]:.3f}, {d["y"]:.3f})')
        self.robmove('PTP', d['x'], d['y'], m['approach_z'], q, m['travel_speed'])
        self.robmove('LIN', d['x'], d['y'], release_z, q, m['fine_speed'])
        self.gripper(0.0)
        time.sleep(1.0)
        self.robmove('LIN', d['x'], d['y'], m['approach_z'], q, m['fine_speed'])

        final = self.get_pose(obj)
        if final is None or not self._inside(final, d):
            where = f'({final.x:.3f}, {final.y:.3f}, {final.z:.3f})' if final else 'unknown'
            raise SkillError(f'{obj} did not end up inside {dest_name}; it is at {where}.')
        return f'{obj} is now in {dest_name} at ({final.x:.3f}, {final.y:.3f}).'

    def _inside(self, p, d):
        # Resting inside the footprint, and not still hanging in the gripper above it.
        return (abs(p.x - d['x']) <= d['half_x'] and abs(p.y - d['y']) <= d['half_y']
                and p.z < d['release_z_center'] + cube_half_size())
