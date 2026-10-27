"""M4 shot selector: from the incoming ball at the hit point choose stroke type and target point.

Pure python (no ROS). Rule based, no spin yet (spin comes with M5). Inputs are the predicted ball
state at the hit (position, velocity); outputs are a stroke type, the landing target on the opponent
half and the launch elevations the planner may try (first one that works wins).

  fast ball                    -> BLOCK : use the incoming pace, short return
  low or steeply rising ball   -> LOOP  : lift it with a higher arc, deep
  high ball                    -> DRIVE : flat and deeper than the standard drive
  otherwise                    -> DRIVE : medium depth
(All cross court; machine balls reach the hit point ~0.6 s after launch at 3-4 m/s, rising.)

Cross court = to the side opposite of the ball's lateral position at the hit; a ball near the
middle goes to the middle. The target is kept inside a safe margin of the table.
"""
from dataclasses import dataclass

import numpy as np

# Same numbers as StrokePlan.msg (kept here so the module works without ROS)
TOPSPIN_DRIVE, LOOP, BLOCK = 0, 1, 2
NAMES = {TOPSPIN_DRIVE: 'DRIVE', LOOP: 'LOOP', BLOCK: 'BLOCK'}


@dataclass
class SelectorParams:
    fast_speed: float = 3.5         # |v| at the hit [m/s] from which the ball is blocked
    high_z: float = 1.05            # hit height from which the ball is driven flat [m]
    low_z: float = 0.93             # hit height up to which the ball is looped [m]
    steep_vz: float = 1.8           # rising speed from which the ball is looped [m/s]
    y_gain: float = 0.40            # cross court: target y = -y_gain * clip(y_hit / y_scale)
    y_scale: float = 0.3
    y_max: float = 0.55             # largest |target y| [m] (table half width 0.7625)
    depth: dict = None              # stroke type -> target x [m] (negative = opponent half)
    elevations: dict = None         # stroke type -> launch elevations to try [deg]

    def __post_init__(self):
        if self.depth is None:
            self.depth = {BLOCK: -0.5, TOPSPIN_DRIVE: -0.85, LOOP: -1.05}
        if self.elevations is None:
            self.elevations = {BLOCK: (25.0, 30.0, 20.0, 35.0),
                               TOPSPIN_DRIVE: (20.0, 25.0, 30.0, 15.0, 35.0),
                               LOOP: (35.0, 30.0, 40.0, 25.0)}


@dataclass
class ShotChoice:
    stroke_type: int
    target: tuple
    elevations: tuple
    reason: str

    @property
    def name(self):
        return NAMES[self.stroke_type]


def select_shot(hit_pos, v_in, params=None):
    p = params or SelectorParams()
    speed = float(np.linalg.norm(v_in))
    z, y = float(hit_pos[2]), float(hit_pos[1])
    if speed >= p.fast_speed:
        kind, why = BLOCK, f'fast ball {speed:.1f} m/s'
    elif z <= p.low_z or v_in[2] >= p.steep_vz:
        kind, why = LOOP, f'low/rising ball z={z:.2f} m vz={v_in[2]:.1f} m/s'
    elif z >= p.high_z:
        kind, why = TOPSPIN_DRIVE, f'high ball z={z:.2f} m'
    else:
        kind, why = TOPSPIN_DRIVE, f'medium ball {speed:.1f} m/s z={z:.2f} m'
    x_t = p.depth[kind] - (0.15 if z >= p.high_z and kind == TOPSPIN_DRIVE else 0.0)
    y_t = -p.y_gain * float(np.clip(y / p.y_scale, -1.0, 1.0))
    y_t = float(np.clip(y_t, -p.y_max, p.y_max))
    return ShotChoice(kind, (float(x_t), y_t), p.elevations[kind], why)
