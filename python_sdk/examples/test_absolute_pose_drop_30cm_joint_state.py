#!/usr/bin/env python3

import time

import rclpy

from sailor_sdk import (
    AbsolutePoseTOPPRAPlanner,
    DEFAULT_INITIAL_Q,
    NullspaceSolverClient,
    SailorROS2Interface,
    dq_pose_to_quaternion_translation,
    pose_dq_from_quaternion_translation,
)


DROP_DISTANCE_Z = 0.30
WAYPOINT_COUNT = 16
SAMPLE_COUNT = 160
INCLUDE_WAIST = True
KEEP_BODY_Z_VERTICAL = True
INITIAL_HOLD_SECONDS = 1.0
FINAL_HOLD_SECONDS = 2.0


def build_target_pose_from_default_state(solver: NullspaceSolverClient):
    state = solver.get_state(DEFAULT_INITIAL_Q)
    quaternion, translation = dq_pose_to_quaternion_translation(state.absolute_pose)
    translation[2] -= DROP_DISTANCE_Z
    target_pose = pose_dq_from_quaternion_translation(
        qw=quaternion[0],
        qx=quaternion[1],
        qy=quaternion[2],
        qz=quaternion[3],
        x=translation[0],
        y=translation[1],
        z=translation[2],
    )
    return state, target_pose, translation


def publish_hold_state(ros: SailorROS2Interface, q, hold_seconds: float) -> None:
    end_time = time.monotonic() + hold_seconds
    while time.monotonic() < end_time:
        ros.publish_joint_state(q)
        time.sleep(0.1)


def main() -> None:
    rclpy.init()

    ros = SailorROS2Interface(initial_q=DEFAULT_INITIAL_Q)
    solver = NullspaceSolverClient()
    planner = AbsolutePoseTOPPRAPlanner()

    try:
        start_state, target_pose, target_translation = build_target_pose_from_default_state(solver)
        start_quat, start_translation = dq_pose_to_quaternion_translation(start_state.absolute_pose)

        print('start absolute translation =', [round(value, 6) for value in start_translation])
        print('target absolute translation =', [round(value, 6) for value in target_translation])
        print('drop distance z =', DROP_DISTANCE_Z)
        print('include waist =', INCLUDE_WAIST)
        print('keep body z vertical =', KEEP_BODY_Z_VERTICAL)

        plan = planner.plan_absolute_pose(
            solver,
            q_start=DEFAULT_INITIAL_Q,
            target_pose=target_pose,
            include_waist=INCLUDE_WAIST,
            keep_body_z_vertical=KEEP_BODY_Z_VERTICAL,
            waypoint_count=WAYPOINT_COUNT,
            sample_count=SAMPLE_COUNT,
        )

        print(f'planned duration: {plan.trajectory.duration:.3f}s')
        print(f'joint waypoints: {len(plan.joint_waypoints)}')
        print(f'trajectory samples: {len(plan.trajectory.sample_times)}')
        if plan.solve_results:
            print('final solve reason:', plan.solve_results[-1].reason)

        print(f'publishing initial state to /whole/joint_states for {INITIAL_HOLD_SECONDS:.1f}s')
        publish_hold_state(ros, DEFAULT_INITIAL_Q, INITIAL_HOLD_SECONDS)

        print('publishing planned joint trajectory to /whole/joint_states')
        ros.execute_timed_joint_state_trajectory(plan.trajectory)

        print(f'holding final state for {FINAL_HOLD_SECONDS:.1f}s')
        publish_hold_state(ros, plan.trajectory.positions[-1], FINAL_HOLD_SECONDS)
    finally:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
