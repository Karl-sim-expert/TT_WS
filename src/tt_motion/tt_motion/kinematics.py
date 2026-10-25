"""Forward kinematics, Jacobian and damped-least-squares IK for the Panda + paddle chain.

The chain is read from the processed URDF (the same string robot_state_publisher gets), so
there are no hard-coded link offsets: world -> ... -> `tip` (default: the paddle link).
Pure numpy; no ROS imports, so it can be unit-tested anywhere.
"""
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_angle_to_matrix(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def matrix_to_rotvec(r):
    """Rotation vector of a rotation matrix (angle in [0, pi])."""
    cos_a = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-9:
        return np.zeros(3)
    if np.pi - angle < 1e-6:  # near pi: axis from the symmetric part
        w, v = np.linalg.eigh((r + r.T) / 2.0 - np.eye(3) * cos_a)
        axis = v[:, np.argmax(w)]
        return axis * angle
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * np.sin(angle))
    return axis * angle


def matrix_to_quat(r):
    """Quaternion (x, y, z, w)."""
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                         (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(r[i, i] - r[j, j] - r[k, k] + 1.0) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q


def quat_to_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotation_aligning(a, b):
    """Minimal rotation taking unit vector a onto unit vector b."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = float(np.dot(a, b))
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        perp = np.cross(a, [1.0, 0, 0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0, 1.0, 0])
        return axis_angle_to_matrix(perp, np.pi)
    return axis_angle_to_matrix(v, np.arctan2(s, c))


class Chain:
    """Serial chain world -> tip with revolute joints (fixed joints are folded in)."""

    def __init__(self, urdf_xml, tip='paddle', root='world'):
        robot = ET.fromstring(urdf_xml)
        by_child = {}
        for j in robot.findall('joint'):
            by_child[j.find('child').get('link')] = j
        chain = []
        link = tip
        while link != root:
            if link not in by_child:
                raise ValueError(f"link '{link}' has no parent joint (root '{root}' not reached)")
            joint = by_child[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        chain.reverse()

        self.joint_names = []
        self.lower, self.upper, self.vel_limit = [], [], []
        # steps: (fixed transform parent->joint frame, axis or None)
        self._steps = []
        for j in chain:
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            t = np.eye(4)
            t[:3, :3] = rpy_to_matrix(*rpy)
            t[:3, 3] = xyz
            if j.get('type') in ('revolute', 'continuous'):
                axis = [float(v) for v in j.find('axis').get('xyz').split()]
                lim = j.find('limit')
                self.joint_names.append(j.get('name'))
                self.lower.append(float(lim.get('lower')))
                self.upper.append(float(lim.get('upper')))
                self.vel_limit.append(float(lim.get('velocity')))
                self._steps.append((t, np.array(axis, float)))
            else:
                self._steps.append((t, None))
        self.lower = np.array(self.lower)
        self.upper = np.array(self.upper)
        self.vel_limit = np.array(self.vel_limit)
        self.n = len(self.joint_names)

    def fk(self, q, with_frames=False):
        """4x4 world pose of the tip (and the per-joint (origin, axis) in world if requested)."""
        t = np.eye(4)
        k = 0
        frames = []
        for fixed, axis in self._steps:
            t = t @ fixed
            if axis is not None:
                frames.append((t[:3, 3].copy(), t[:3, :3] @ axis))
                rot = np.eye(4)
                rot[:3, :3] = axis_angle_to_matrix(axis, q[k])
                t = t @ rot
                k += 1
        return (t, frames) if with_frames else t

    def jacobian(self, q):
        """6 x n geometric Jacobian at the tip: [linear; angular] in the world frame."""
        t, frames = self.fk(q, with_frames=True)
        p = t[:3, 3]
        jac = np.zeros((6, self.n))
        for i, (origin, axis) in enumerate(frames):
            jac[:3, i] = np.cross(axis, p - origin)
            jac[3:, i] = axis
        return jac

    def ik(self, target, q_seed, max_iter=100, pos_tol=1e-4, rot_tol=1e-3, damping=0.02,
           max_step=0.3, nullspace_pull=0.02, q_pref=None):
        """Damped least squares from q_seed. target: 4x4 world pose. Returns (q, ok).

        A weak secondary task pulls toward q_pref (default q_seed) to resolve the redundancy;
        joint limits are enforced by clamping.
        """
        q = np.array(q_seed, float)
        q_pref = q.copy() if q_pref is None else np.asarray(q_pref, float)
        for _ in range(max_iter):
            cur = self.fk(q)
            err = np.concatenate([
                target[:3, 3] - cur[:3, 3],
                matrix_to_rotvec(target[:3, :3] @ cur[:3, :3].T)])
            if np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < rot_tol:
                return q, True
            jac = self.jacobian(q)
            jjt = jac @ jac.T + damping ** 2 * np.eye(6)
            jpinv = jac.T @ np.linalg.inv(jjt)
            dq = jpinv @ err
            null = (np.eye(self.n) - jpinv @ jac) @ (nullspace_pull * (q_pref - q))
            dq = dq + null
            norm = np.max(np.abs(dq))
            if norm > max_step:
                dq *= max_step / norm
            q = np.clip(q + dq, self.lower, self.upper)
        cur = self.fk(q)
        err_p = np.linalg.norm(target[:3, 3] - cur[:3, 3])
        err_r = np.linalg.norm(matrix_to_rotvec(target[:3, :3] @ cur[:3, :3].T))
        return q, bool(err_p < 5 * pos_tol and err_r < 5 * rot_tol)

    def joint_velocity(self, q, twist, damping=0.01):
        """Joint velocity that gives the tip the 6D twist [v; omega] (damped pseudo-inverse)."""
        jac = self.jacobian(q)
        jjt = jac @ jac.T + damping ** 2 * np.eye(6)
        return jac.T @ np.linalg.solve(jjt, np.asarray(twist, float))
