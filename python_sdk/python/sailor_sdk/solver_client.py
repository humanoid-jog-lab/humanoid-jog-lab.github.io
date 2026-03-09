from __future__ import annotations

import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

from .constants import DEFAULT_SOLVER_BINARY, DEFAULT_TOPPRA_MAX_ACC, DEFAULT_TOPPRA_MAX_VEL, HUMANOID_JOINT_NAMES


def pose_dq_from_quaternion_translation(
    *,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
    x: float,
    y: float,
    z: float,
) -> List[float]:
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1e-12:
        raise ValueError('Quaternion norm must be positive')

    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    dw = -0.5 * (x * qx + y * qy + z * qz)
    dx = 0.5 * (x * qw + y * qz - z * qy)
    dy = 0.5 * (-x * qz + y * qw + z * qx)
    dz = 0.5 * (x * qy - y * qx + z * qw)

    return [qw, qx, qy, qz, dw, dx, dy, dz]


@dataclass
class SolverState:
    absolute_pose: List[float]
    relative_pose: List[float]
    left_pose: List[float]
    right_pose: List[float]
    joint_positions: List[float]
    joint_names: List[str]


@dataclass
class SolveResult:
    q: List[float]
    reason: str
    converged: bool
    apply_result: bool
    final_command_norm: float
    final_invariant_norm: float


@dataclass
class RetimedTrajectory:
    duration: float
    waypoint_times: List[float]
    sample_times: List[float]
    positions: List[List[float]]
    velocities: List[List[float]]
    accelerations: List[List[float]]
    gridpoints: List[float]
    grid_times: List[float]
    path_velocity_squared: List[float]


class NullspaceSolverClient:
    def __init__(self, solver_binary: str | os.PathLike[str] | None = None, urdf_path: str | os.PathLike[str] | None = None):
        repo_root = Path(__file__).resolve().parents[3]
        default_binary = repo_root / DEFAULT_SOLVER_BINARY
        self.solver_binary = Path(solver_binary) if solver_binary else default_binary
        self.urdf_path = Path(urdf_path) if urdf_path else None

    def get_state(self, q: Sequence[float]) -> SolverState:
        joint_positions = [float(value) for value in q]
        result = self._run_solver('state', q=q)
        return SolverState(
            absolute_pose=_parse_csv_floats(result['absolute_pose']),
            relative_pose=_parse_csv_floats(result['relative_pose']),
            left_pose=_parse_csv_floats(result['left_pose']),
            right_pose=_parse_csv_floats(result['right_pose']),
            joint_positions=joint_positions,
            joint_names=list(HUMANOID_JOINT_NAMES),
        )

    def get_absolute_pose(self, q: Sequence[float]) -> List[float]:
        return self.get_state(q).absolute_pose

    def get_relative_pose(self, q: Sequence[float]) -> List[float]:
        return self.get_state(q).relative_pose

    def solve_absolute(
        self,
        *,
        q: Sequence[float],
        target_pose: Sequence[float],
        include_waist: bool = False,
        keep_body_z_vertical: bool = False,
        joint_limit_avoidance: bool = True,
    ) -> SolveResult:
        result = self._run_solver(
            'absolute',
            q=q,
            target=target_pose,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
        )
        return self._build_solve_result(result)

    def solve_relative(
        self,
        *,
        q: Sequence[float],
        target_pose: Sequence[float],
        include_waist: bool = False,
        keep_body_z_vertical: bool = False,
        joint_limit_avoidance: bool = True,
    ) -> SolveResult:
        result = self._run_solver(
            'relative',
            q=q,
            target=target_pose,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
        )
        return self._build_solve_result(result)

    def retime_waypoints(
        self,
        waypoints: Sequence[Sequence[float]],
        *,
        max_vel: float | Sequence[float] = DEFAULT_TOPPRA_MAX_VEL,
        max_acc: float | Sequence[float] = DEFAULT_TOPPRA_MAX_ACC,
        gridpoints_hint: int = 0,
        sample_count: int = 0,
    ) -> RetimedTrajectory:
        result = self._run_solver(
            'retime',
            q=[0.0] * len(waypoints[0]),
            waypoints=waypoints,
            max_vel=max_vel,
            max_acc=max_acc,
            gridpoints_hint=gridpoints_hint,
            sample_count=sample_count,
        )
        return RetimedTrajectory(
            duration=float(result['duration']),
            waypoint_times=_parse_csv_floats(result['waypoint_times']),
            sample_times=_parse_csv_floats(result['sample_times']),
            positions=_parse_matrix(result['positions']),
            velocities=_parse_matrix(result['velocities']),
            accelerations=_parse_matrix(result['accelerations']),
            gridpoints=_parse_csv_floats(result.get('gridpoints', '')),
            grid_times=_parse_csv_floats(result.get('grid_times', '')),
            path_velocity_squared=_parse_csv_floats(result.get('path_velocity_squared', '')),
        )

    def _build_solve_result(self, result: Dict[str, str]) -> SolveResult:
        return SolveResult(
            q=_parse_csv_floats(result['q']),
            reason=result['reason'],
            converged=_parse_bool(result['converged']),
            apply_result=_parse_bool(result['apply_result']),
            final_command_norm=float(result['final_command_norm']),
            final_invariant_norm=float(result['final_invariant_norm']),
        )

    def _run_solver(
        self,
        mode: str,
        *,
        q: Sequence[float],
        target: Sequence[float] | None = None,
        include_waist: bool | None = None,
        keep_body_z_vertical: bool | None = None,
        joint_limit_avoidance: bool | None = None,
        waypoints: Sequence[Sequence[float]] | None = None,
        max_vel: float | Sequence[float] | None = None,
        max_acc: float | Sequence[float] | None = None,
        gridpoints_hint: int | None = None,
        sample_count: int | None = None,
    ) -> Dict[str, str]:
        command = [
            str(self.solver_binary),
            '--mode',
            mode,
        ]
        if mode != 'retime':
            command.extend(['--q', _format_csv(q)])
        if target is not None:
            command.extend(['--target', _format_csv(target)])
        if include_waist is not None:
            command.extend(['--include-waist', '1' if include_waist else '0'])
        if keep_body_z_vertical is not None:
            command.extend(['--keep-body-z-vertical', '1' if keep_body_z_vertical else '0'])
        if joint_limit_avoidance is not None:
            command.extend(['--joint-limit-avoidance', '1' if joint_limit_avoidance else '0'])
        if waypoints is not None:
            command.extend(['--waypoints', _format_matrix(waypoints)])
        if max_vel is not None:
            command.extend(['--max-vel', _format_limit(max_vel)])
        if max_acc is not None:
            command.extend(['--max-acc', _format_limit(max_acc)])
        if gridpoints_hint is not None:
            command.extend(['--gridpoints-hint', str(int(gridpoints_hint))])
        if sample_count is not None:
            command.extend(['--sample-count', str(int(sample_count))])
        if self.urdf_path is not None:
            command.extend(['--urdf-path', str(self.urdf_path)])

        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip() or completed.stdout.strip() or f'Solver exited with {completed.returncode}')

        parsed = _parse_key_value_output(completed.stdout)
        if parsed.get('status') != 'ok':
            raise RuntimeError(parsed.get('message', 'Solver returned a non-ok status'))
        return parsed


def _format_csv(values: Sequence[float]) -> str:
    return ','.join(f'{float(value):.17g}' for value in values)


def _format_matrix(rows: Sequence[Sequence[float]]) -> str:
    return ';'.join(_format_csv(row) for row in rows)


def _format_limit(limit: float | Sequence[float]) -> str:
    if isinstance(limit, (int, float)):
        return f'{float(limit):.17g}'
    return _format_csv(limit)


def _parse_csv_floats(text: str) -> List[float]:
    if not text:
        return []
    return [float(item) for item in text.split(',') if item]


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _parse_matrix(text: str) -> List[List[float]]:
    if not text:
        return []
    return [_parse_csv_floats(row) for row in text.split(';') if row]


def _parse_key_value_output(stdout: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for raw_line in stdout.splitlines():
        line = raw_line.strip()
        if not line or '=' not in line:
            continue
        key, value = line.split('=', 1)
        result[key.strip()] = value.strip()
    return result
