"""Keep the arm out of the table (the simulator has no collision geometry for it, so a stroke that
sweeps through the table would silently pass through it).

Pure numpy. The arm is approximated by the segments between the joint origins (+ the paddle disc),
the rail carriage by its footprint. Rule: inside the table footprint (expanded by `margin_xy`)
every arm point must stay above the table top by a clearance (link thickness).
"""
import numpy as np

TABLE_HALF_X = 1.37
TABLE_HALF_Y = 0.7625
TABLE_TOP = 0.76


class TableGuard:
    def __init__(self, chain, margin_xy=0.04, arm_clearance=0.08, paddle_clearance=0.02,
                 carriage_half=0.2, paddle_radius=0.075):
        self.chain = chain
        self.margin_xy = margin_xy
        self.arm_clearance = arm_clearance
        self.paddle_clearance = paddle_clearance
        self.carriage_half = carriage_half
        self.paddle_radius = paddle_radius
        names = chain.joint_names
        self.i_base = names.index('panda_joint1') if 'panda_joint1' in names else None

    def _inside(self, p):
        return (abs(p[0]) < TABLE_HALF_X + self.margin_xy
                and abs(p[1]) < TABLE_HALF_Y + self.margin_xy)

    def check(self, q):
        """None if the pose is clear, else a short description of the first violation."""
        t, frames = self.chain.fk(q, with_frames=True)
        origins = [f[0] for f in frames]
        if self.i_base is not None:
            base = origins[self.i_base]
            if (base[0] - self.carriage_half < TABLE_HALF_X + self.margin_xy
                    and abs(base[1]) < TABLE_HALF_Y + self.margin_xy + self.carriage_half):
                return f'rail carriage reaches the table (arm base at x={base[0]:.2f}, y={base[1]:.2f})'
            origins = origins[self.i_base:]
        pts = origins + [t[:3, 3]]
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.05)) + 1)
            for s in np.linspace(0.0, 1.0, n):
                p = a + s * (b - a)
                if self._inside(p) and p[2] < TABLE_TOP + self.arm_clearance:
                    return f'arm link at ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) is inside the table'
        # paddle disc rim
        c, nrm = t[:3, 3], t[:3, 2]
        u = np.cross(nrm, [0.0, 0.0, 1.0])
        u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1.0, 0.0, 0.0])
        w = np.cross(nrm, u)
        for ang in np.linspace(0.0, 2 * np.pi, 12, endpoint=False):
            p = c + self.paddle_radius * (np.cos(ang) * u + np.sin(ang) * w)
            if self._inside(p) and p[2] < TABLE_TOP + self.paddle_clearance:
                return f'paddle at ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) is inside the table'
        return None

    def first_violation(self, qs, step=1):
        for i in range(0, len(qs), step):
            msg = self.check(qs[i])
            if msg:
                return i, msg
        return None
