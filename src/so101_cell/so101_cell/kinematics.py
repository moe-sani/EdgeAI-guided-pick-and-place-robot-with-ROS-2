"""Forward and inverse kinematics of the SO-101 arm, for top-down grasps (numpy only).

The chain is read from the URDF, so sim and real use the same geometry. The arm has 5 joints, which is
exactly enough for "gripper pointing straight down at (x, y, z) with yaw psi": shoulder_pan picks the
vertical plane, shoulder_lift/elbow_flex/wrist_flex place the wrist and keep the tool vertical, and
wrist_roll turns the jaws about the (vertical) tool axis.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIPPER_JOINT = 'gripper'


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = _rpy(*rpy)
    m[:3, 3] = xyz
    return m


def _rot_axis(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    m = np.eye(4)
    m[:3, :3] = [[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                 [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                 [z * x * v - y * s, z * y * v + x * s, c + z * z * v]]
    return m


def yaw_of(rot):
    """Heading of the jaw-closing direction (tool x axis) projected on the table."""
    return math.atan2(rot[1, 0], rot[0, 0])


def down_rotation(yaw):
    """Tool frame pointing straight down (tool z = -world z), tool x along ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


class IKError(Exception):
    pass


class SO101Kinematics:
    """Chain base_link -> gripper_link plus a fixed tool frame (the grasp centre between the jaws)."""

    def __init__(self, urdf_xml, base_link='base_link', tip_link='gripper_link', tool_xyz=(0.010, 0.0, -0.085),
                 min_height=0.01):
        root = ET.fromstring(urdf_xml)
        joints = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], tip_link
        while link != base_link:
            j = joints[link]
            chain.append(j)
            link = j.find('parent').get('link')
        self.joints = []  # (name, fixed transform, axis, lower, upper)
        for j in reversed(chain):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
            ax = j.find('axis')
            axis = [float(v) for v in (ax.get('xyz') if ax is not None else '0 0 1').split()]
            lim = j.find('limit')
            lo, hi = (float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else (-math.pi, math.pi)
            self.joints.append((j.get('name'), _tf(xyz, rpy), axis, lo, hi))
        names = [j[0] for j in self.joints]
        if names != ARM_JOINTS:
            raise ValueError(f'unexpected SO-101 chain {names}')
        self.min_height = min_height  # lowest the elbow/wrist may go (m above the table), see joints_clear()
        self.lower = np.array([j[3] for j in self.joints])
        self.upper = np.array([j[4] for j in self.joints])
        # Tool frame: at tool_xyz in gripper_link, z pointing out of the jaws (gripper_link z points back
        # into the wrist), x along the jaw-closing direction (gripper_link x).
        self.tool = _tf(tool_xyz, (0.0, math.pi, 0.0))
        self.tool[:3, :3] = np.diag([1.0, -1.0, -1.0])

    @classmethod
    def from_file(cls, path, **kw):
        with open(path) as f:
            return cls(f.read(), **kw)

    def fk(self, q):
        """4x4 pose of the tool frame in base_link for arm joint angles ``q`` (5 values, rad)."""
        m = np.eye(4)
        for (_, fixed, axis, _, _), angle in zip(self.joints, q):
            m = m @ fixed @ _rot_axis(axis, angle)
        return m @ self.tool

    def joints_clear(self, q):
        """True if the elbow and wrist joints are above ``min_height`` (keeps the arm off the table)."""
        m = np.eye(4)
        for i, ((_, fixed, axis, _, _), angle) in enumerate(zip(self.joints, q)):
            m = m @ fixed @ _rot_axis(axis, angle)
            if i >= 2 and m[2, 3] < self.min_height:
                return False
        return True

    def _jacobian(self, q, eps=1e-6):
        base = self.fk(q)
        jac = np.zeros((6, len(q)))
        for i in range(len(q)):
            dq = np.array(q, dtype=float)
            dq[i] += eps
            m = self.fk(dq)
            jac[:3, i] = (m[:3, 3] - base[:3, 3]) / eps
            dr = m[:3, :3] @ base[:3, :3].T
            jac[3:, i] = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / (2 * eps)
        return base, jac

    @staticmethod
    def _error(m, pos, rot):
        dp = pos - m[:3, 3]
        dr = rot @ m[:3, :3].T
        angle = math.acos(max(-1.0, min(1.0, (np.trace(dr) - 1) / 2)))
        w = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])
        n = np.linalg.norm(w)
        dw = w / n * angle if n > 1e-9 else np.zeros(3)
        return np.concatenate([dp, dw])

    def solve(self, pos, rot, seed, iters=200, tol_pos=5e-4, tol_rot=1e-2):
        """Damped least squares IK from ``seed``; returns joint angles within limits or raises IKError."""
        q = np.clip(np.array(seed, dtype=float), self.lower, self.upper)
        pos = np.asarray(pos, dtype=float)
        weights = np.array([1, 1, 1, 0.1, 0.1, 0.1])  # metres vs radians
        for _ in range(iters):
            m, jac = self._jacobian(q)
            err = self._error(m, pos, rot)
            if np.linalg.norm(err[:3]) < tol_pos and np.linalg.norm(err[3:]) < tol_rot:
                if not self.joints_clear(q):
                    raise IKError(f'IK solution for ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) hits the table')
                return q
            jw = jac * weights[:, None]
            ew = err * weights
            dq = jw.T @ np.linalg.solve(jw @ jw.T + 1e-6 * np.eye(6), ew)
            step = np.max(np.abs(dq))
            if step > 0.3:
                dq *= 0.3 / step
            q = np.clip(q + dq, self.lower, self.upper)
        raise IKError(f'no IK solution for ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')

    # Starting guesses for the top-down IK (the solver is local); the current joints are tried first.
    SEEDS = ([0.0, -1.0, 1.0, 1.0, 0.0], [0.0, -0.5, 0.5, 1.2, 0.0], [0.0, 0.0, 0.5, 1.0, 0.0],
             [0.0, -1.5, 1.5, 0.5, 0.0], [0.0, 0.5, -0.5, 1.5, 0.0])

    def solve_down(self, pos, yaw, seed, symmetry=math.pi / 2):
        """Top-down IK at ``pos`` with the jaws closing along ``yaw`` (mod ``symmetry``, e.g. 90 deg for a cube).

        Tries every equivalent yaw and returns the solution closest to ``seed`` (smallest joint move).
        """
        best, last_err = None, IKError(f'({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) is out of reach')
        seed = np.asarray(seed, dtype=float)
        pan = math.atan2(pos[1], pos[0])
        n = max(1, int(round(2 * math.pi / symmetry)))
        for k in range(n):
            rot = down_rotation(yaw + k * symmetry)
            for s in [seed] + [np.array([pan] + list(g[1:])) for g in self.SEEDS]:
                try:
                    q = self.solve(pos, rot, s)
                except IKError as e:
                    last_err = e
                    continue
                if best is None or np.linalg.norm(q - seed) < np.linalg.norm(best - seed):
                    best = q
                break
        if best is None:
            raise last_err
        return best

    def line(self, q_start, pos_end, steps):
        """Joint waypoints for a straight tool line from fk(q_start) to ``pos_end``, keeping the orientation."""
        start = self.fk(q_start)
        rot, p0 = start[:3, :3], start[:3, 3]
        out, q = [], np.array(q_start, dtype=float)
        for i in range(1, steps + 1):
            q = self.solve(p0 + (np.asarray(pos_end) - p0) * i / steps, rot, q)
            out.append(q)
        return out
