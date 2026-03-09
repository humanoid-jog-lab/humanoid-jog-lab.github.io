from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

from .constants import HUMANOID_JOINT_NAMES, LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES


def _coerce_q(q: Sequence[float], minimum_length: int = 18) -> List[float]:
    values = list(q)
    if len(values) < minimum_length:
        raise ValueError(f'Expected at least {minimum_length} joint values, got {len(values)}')
    return [float(value) for value in values]


def build_joint_state_payload(
    q: Sequence[float],
    *,
    velocities: Sequence[float] | None = None,
    effort: Sequence[float] | None = None,
) -> dict:
    q_values = _coerce_q(q, minimum_length=len(HUMANOID_JOINT_NAMES))
    velocity_values = [0.0] * len(HUMANOID_JOINT_NAMES) if velocities is None else [float(value) for value in velocities]
    effort_values = [0.0] * len(HUMANOID_JOINT_NAMES) if effort is None else [float(value) for value in effort]

    if len(velocity_values) != len(HUMANOID_JOINT_NAMES):
        raise ValueError(f'Expected {len(HUMANOID_JOINT_NAMES)} velocity values, got {len(velocity_values)}')
    if len(effort_values) != len(HUMANOID_JOINT_NAMES):
        raise ValueError(f'Expected {len(HUMANOID_JOINT_NAMES)} effort values, got {len(effort_values)}')

    return {
        'name': list(HUMANOID_JOINT_NAMES),
        'position': q_values[:len(HUMANOID_JOINT_NAMES)],
        'velocity': velocity_values,
        'effort': effort_values,
    }


@dataclass
class RobotArmServoCommand:
    q: List[float]
    run_mode: int = 1
    robot_power: bool = True
    run_time: float = 0.0

    @classmethod
    def from_q(
        cls,
        q: Sequence[float],
        run_mode: int = 1,
        robot_power: bool = True,
        run_time: float = 0.0,
    ) -> 'RobotArmServoCommand':
        return cls(q=_coerce_q(q), run_mode=run_mode, robot_power=robot_power, run_time=run_time)

    def as_dict(self) -> dict:
        message = {
            'run_mode': int(self.run_mode),
            'robot_power': bool(self.robot_power),
            'run_time': float(self.run_time),
        }
        for index, name in enumerate(LEFT_ARM_JOINT_NAMES, start=4):
            message[f'{name}_position'] = float(self.q[index])
        for offset, name in enumerate(RIGHT_ARM_JOINT_NAMES, start=11):
            message[f'{name}_position'] = float(self.q[offset])
        return message


@dataclass
class RobotServoCommand:
    q: List[float]
    run_mode: int = 0
    x: float = 0.0
    y: float = 0.0
    phi: float = 0.0
    mode_leg_select: int = 0
    vel: float = 0.3

    @classmethod
    def from_q(
        cls,
        q: Sequence[float],
        run_mode: int = 0,
        x: float = 0.0,
        y: float = 0.0,
        phi: float = 0.0,
        mode_leg_select: int = 0,
        vel: float = 0.3,
    ) -> 'RobotServoCommand':
        return cls(
            q=_coerce_q(q),
            run_mode=run_mode,
            x=x,
            y=y,
            phi=phi,
            mode_leg_select=mode_leg_select,
            vel=vel,
        )

    def as_dict(self) -> dict:
        return {
            'run_mode': int(self.run_mode),
            'ankle_position': float(self.q[0]),
            'knee_position': float(self.q[1]),
            'hip_pitch_position': float(self.q[2]),
            'hip_yaw_position': float(self.q[3]),
            'x': float(self.x),
            'y': float(self.y),
            'phi': float(self.phi),
            'mode_leg_select': int(self.mode_leg_select),
            'vel': float(self.vel),
        }
