from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

from .constants import DEFAULT_TOPPRA_MAX_ACC, DEFAULT_TOPPRA_MAX_VEL
from .solver_client import RetimedTrajectory, SolveResult, SolverState


@dataclass
class AbsolutePosePlan:
    start_state: SolverState
    final_state: SolverState
    target_pose: List[float]
    pose_waypoints: List[List[float]]
    joint_waypoints: List[List[float]]
    solve_results: List[SolveResult]
    trajectory: RetimedTrajectory


@dataclass
class RelativePosePlan:
    start_state: SolverState
    final_state: SolverState
    target_pose: List[float]
    pose_waypoints: List[List[float]]
    joint_waypoints: List[List[float]]
    solve_results: List[SolveResult]
    trajectory: RetimedTrajectory


class AbsolutePoseTOPPRAPlanner:
    def plan_absolute_pose(
        self,
        solver,
        *,
        q_start: Sequence[float],
        target_pose: Sequence[float],
        include_waist: bool = False,
        keep_body_z_vertical: bool = False,
        joint_limit_avoidance: bool = True,
        waypoint_count: int | None = None,
        max_vel: float | Sequence[float] = DEFAULT_TOPPRA_MAX_VEL,
        max_acc: float | Sequence[float] = DEFAULT_TOPPRA_MAX_ACC,
        gridpoints_hint: int = 0,
        sample_count: int = 0,
    ) -> AbsolutePosePlan:
        q_current = [float(value) for value in q_start]
        start_state = solver.get_state(q_current)
        target_pose_list, pose_waypoints, joint_waypoints, solve_results, trajectory, q_final = _plan_pose_waypoints(
            solver,
            q_start=q_current,
            start_pose=start_state.absolute_pose,
            target_pose=target_pose,
            solve_fn=solver.solve_absolute,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
            waypoint_count=waypoint_count,
            max_vel=max_vel,
            max_acc=max_acc,
            gridpoints_hint=gridpoints_hint,
            sample_count=sample_count,
            failure_label='Absolute',
        )
        final_state = solver.get_state(q_final)
        return AbsolutePosePlan(
            start_state=start_state,
            final_state=final_state,
            target_pose=target_pose_list,
            pose_waypoints=pose_waypoints,
            joint_waypoints=joint_waypoints,
            solve_results=solve_results,
            trajectory=trajectory,
        )

    def _estimate_waypoint_count(self, start_pose: Sequence[float], target_pose: Sequence[float]) -> int:
        return estimate_pose_waypoint_count(start_pose, target_pose)


class RelativePoseTOPPRAPlanner:
    def plan_relative_pose(
        self,
        solver,
        *,
        q_start: Sequence[float],
        target_pose: Sequence[float],
        include_waist: bool = False,
        keep_body_z_vertical: bool = False,
        joint_limit_avoidance: bool = True,
        waypoint_count: int | None = None,
        max_vel: float | Sequence[float] = DEFAULT_TOPPRA_MAX_VEL,
        max_acc: float | Sequence[float] = DEFAULT_TOPPRA_MAX_ACC,
        gridpoints_hint: int = 0,
        sample_count: int = 0,
    ) -> RelativePosePlan:
        q_current = [float(value) for value in q_start]
        start_state = solver.get_state(q_current)
        target_pose_list, pose_waypoints, joint_waypoints, solve_results, trajectory, q_final = _plan_pose_waypoints(
            solver,
            q_start=q_current,
            start_pose=start_state.relative_pose,
            target_pose=target_pose,
            solve_fn=solver.solve_relative,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
            waypoint_count=waypoint_count,
            max_vel=max_vel,
            max_acc=max_acc,
            gridpoints_hint=gridpoints_hint,
            sample_count=sample_count,
            failure_label='Relative',
        )
        final_state = solver.get_state(q_final)
        return RelativePosePlan(
            start_state=start_state,
            final_state=final_state,
            target_pose=target_pose_list,
            pose_waypoints=pose_waypoints,
            joint_waypoints=joint_waypoints,
            solve_results=solve_results,
            trajectory=trajectory,
        )

    def _estimate_waypoint_count(self, start_pose: Sequence[float], target_pose: Sequence[float]) -> int:
        return estimate_pose_waypoint_count(start_pose, target_pose)


def _plan_pose_waypoints(
    solver,
    *,
    q_start: Sequence[float],
    start_pose: Sequence[float],
    target_pose: Sequence[float],
    solve_fn,
    include_waist: bool,
    keep_body_z_vertical: bool,
    joint_limit_avoidance: bool,
    waypoint_count: int | None,
    max_vel: float | Sequence[float],
    max_acc: float | Sequence[float],
    gridpoints_hint: int,
    sample_count: int,
    failure_label: str,
):
    q_current = [float(value) for value in q_start]
    target_pose_list = [float(value) for value in target_pose]
    waypoint_count_final = max(2, waypoint_count or estimate_pose_waypoint_count(start_pose, target_pose_list))
    pose_waypoints = interpolate_pose_waypoints(start_pose, target_pose_list, waypoint_count_final)

    joint_waypoints = [q_current[:]]
    solve_results: List[SolveResult] = []

    for pose_target in pose_waypoints[1:]:
        solve_result = solve_fn(
            q=q_current,
            target_pose=pose_target,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
        )
        solve_results.append(solve_result)
        if not solve_result.apply_result:
            raise RuntimeError(f'{failure_label} waypoint solve failed: {solve_result.reason}')
        q_current = solve_result.q[:]
        joint_waypoints.append(q_current[:])

    trajectory = solver.retime_waypoints(
        joint_waypoints,
        max_vel=max_vel,
        max_acc=max_acc,
        gridpoints_hint=gridpoints_hint,
        sample_count=sample_count,
    )
    return target_pose_list, pose_waypoints, joint_waypoints, solve_results, trajectory, q_current


def estimate_pose_waypoint_count(start_pose: Sequence[float], target_pose: Sequence[float]) -> int:
    start_quat, start_xyz = dq_pose_to_quaternion_translation(start_pose)
    target_quat, target_xyz = dq_pose_to_quaternion_translation(target_pose)
    distance = math.sqrt(sum((target_xyz[i] - start_xyz[i]) ** 2 for i in range(3)))
    angle = quaternion_distance(start_quat, target_quat)
    count_from_translation = int(math.ceil(distance / 0.02))
    count_from_rotation = int(math.ceil(angle / math.radians(5.0)))
    return max(2, 1 + max(count_from_translation, count_from_rotation))


def build_relative_distance_target(current_relative_pose: Sequence[float], distance_delta: float) -> tuple[List[float], float, float]:
    quaternion, translation = dq_pose_to_quaternion_translation(current_relative_pose)
    current_distance = math.sqrt(sum(component * component for component in translation))
    target_distance = max(0.0, current_distance - max(0.0, float(distance_delta)))

    if current_distance <= 1e-12 or target_distance <= 1e-12:
        target_translation = [0.0, 0.0, 0.0]
    else:
        scale = target_distance / current_distance
        target_translation = [scale * component for component in translation]

    target_pose = quaternion_translation_to_dq(quaternion, target_translation)
    return target_pose, current_distance, target_distance


def dq_pose_to_quaternion_translation(pose_dq: Sequence[float]) -> tuple[List[float], List[float]]:
    if len(pose_dq) != 8:
        raise ValueError(f'Expected 8 values for pose dual quaternion, got {len(pose_dq)}')
    qr = [float(value) for value in pose_dq[:4]]
    qd = [float(value) for value in pose_dq[4:8]]
    norm = math.sqrt(sum(value * value for value in qr))
    if norm <= 1e-12:
        raise ValueError('Rotation quaternion norm must be positive')
    qr = [value / norm for value in qr]

    qd_conj = [qd[0], -qd[1], -qd[2], -qd[3]]
    translation_quat = quaternion_multiply(qd, quaternion_conjugate(qr))
    translation = [2.0 * translation_quat[1], 2.0 * translation_quat[2], 2.0 * translation_quat[3]]
    return qr, translation


def quaternion_translation_to_dq(quaternion: Sequence[float], translation: Sequence[float]) -> List[float]:
    qw, qx, qy, qz = [float(value) for value in quaternion]
    tx, ty, tz = [float(value) for value in translation]
    dw = -0.5 * (tx * qx + ty * qy + tz * qz)
    dx = 0.5 * (tx * qw + ty * qz - tz * qy)
    dy = 0.5 * (-tx * qz + ty * qw + tz * qx)
    dz = 0.5 * (tx * qy - ty * qx + tz * qw)
    return [qw, qx, qy, qz, dw, dx, dy, dz]


def interpolate_pose_waypoints(start_pose: Sequence[float], target_pose: Sequence[float], waypoint_count: int) -> List[List[float]]:
    if waypoint_count < 2:
        raise ValueError('waypoint_count must be at least 2')
    start_quat, start_xyz = dq_pose_to_quaternion_translation(start_pose)
    target_quat, target_xyz = dq_pose_to_quaternion_translation(target_pose)

    waypoints: List[List[float]] = []
    for index in range(waypoint_count):
        alpha = index / (waypoint_count - 1)
        quat = quaternion_slerp(start_quat, target_quat, alpha)
        xyz = [
            (1.0 - alpha) * start_xyz[axis] + alpha * target_xyz[axis]
            for axis in range(3)
        ]
        waypoints.append(quaternion_translation_to_dq(quat, xyz))
    return waypoints


def interpolate_absolute_pose_waypoints(start_pose: Sequence[float], target_pose: Sequence[float], waypoint_count: int) -> List[List[float]]:
    return interpolate_pose_waypoints(start_pose, target_pose, waypoint_count)


def interpolate_relative_pose_waypoints(start_pose: Sequence[float], target_pose: Sequence[float], waypoint_count: int) -> List[List[float]]:
    return interpolate_pose_waypoints(start_pose, target_pose, waypoint_count)


def quaternion_conjugate(quaternion: Sequence[float]) -> List[float]:
    return [float(quaternion[0]), -float(quaternion[1]), -float(quaternion[2]), -float(quaternion[3])]


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> List[float]:
    lw, lx, ly, lz = [float(value) for value in left]
    rw, rx, ry, rz = [float(value) for value in right]
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def quaternion_distance(a: Sequence[float], b: Sequence[float]) -> float:
    dot = sum(float(a[index]) * float(b[index]) for index in range(4))
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.acos(dot)


def quaternion_slerp(start: Sequence[float], end: Sequence[float], alpha: float) -> List[float]:
    start_q = [float(value) for value in start]
    end_q = [float(value) for value in end]
    dot = sum(start_q[index] * end_q[index] for index in range(4))
    if dot < 0.0:
        end_q = [-value for value in end_q]
        dot = -dot

    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        mixed = [(1.0 - alpha) * start_q[index] + alpha * end_q[index] for index in range(4)]
        norm = math.sqrt(sum(value * value for value in mixed))
        return [value / norm for value in mixed]

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)

    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return [s0 * start_q[index] + s1 * end_q[index] for index in range(4)]
