#!/usr/bin/env python3

import math
import time

import rclpy

from sailor_sdk import (
    DEFAULT_INITIAL_Q,
    NullspaceSolverClient,
    RelativePoseTOPPRAPlanner,
    SailorROS2Interface,
    build_relative_distance_target,
    dq_pose_to_quaternion_translation,
)


RELATIVE_DISTANCE_DELTA = 0.05
WAYPOINT_COUNT = 12
SAMPLE_COUNT = 120
INITIAL_HOLD_SECONDS = 1.0
FINAL_HOLD_SECONDS = 2.0


def build_target_pose_from_default_state(solver: NullspaceSolverClient):
    state = solver.get_state(DEFAULT_INITIAL_Q)
    target_pose, current_distance, target_distance = build_relative_distance_target(
        state.relative_pose,
        RELATIVE_DISTANCE_DELTA,
    )
    return state, target_pose, current_distance, target_distance


def publish_hold_state(ros: SailorROS2Interface, q, hold_seconds: float) -> None:
    end_time = time.monotonic() + hold_seconds
    while time.monotonic() < end_time:
        ros.publish_joint_state(q)
        time.sleep(0.1)


def translation_norm(translation) -> float:
    return math.sqrt(sum(float(component) * float(component) for component in translation))


def main() -> None:
    rclpy.init()

    ros = SailorROS2Interface(initial_q=DEFAULT_INITIAL_Q)
    solver = NullspaceSolverClient()
    planner = RelativePoseTOPPRAPlanner()

    try:
        start_state, target_pose, current_distance, target_distance = build_target_pose_from_default_state(solver)
        _, start_translation = dq_pose_to_quaternion_translation(start_state.relative_pose)
        _, target_translation = dq_pose_to_quaternion_translation(target_pose)

        print('start relative translation =', [round(value, 6) for value in start_translation])
        print('target relative translation =', [round(value, 6) for value in target_translation])
        print('current relative distance =', round(current_distance, 6))
        print('target relative distance =', round(target_distance, 6))
        print('distance delta =', RELATIVE_DISTANCE_DELTA)

        plan = planner.plan_relative_pose(
            solver,
            q_start=DEFAULT_INITIAL_Q,
            target_pose=target_pose,
            include_waist=False,
            waypoint_count=WAYPOINT_COUNT,
            sample_count=SAMPLE_COUNT,
        )

        print(f'planned duration: {plan.trajectory.duration:.3f}s')
        print(f'joint waypoints: {len(plan.joint_waypoints)}')
        print(f'trajectory samples: {len(plan.trajectory.sample_times)}')
        if plan.solve_results:
            print('final solve reason:', plan.solve_results[-1].reason)

        final_state = solver.get_state(plan.trajectory.positions[-1])
        _, final_translation = dq_pose_to_quaternion_translation(final_state.relative_pose)
        print('final relative translation =', [round(value, 6) for value in final_translation])
        print('final relative distance =', round(translation_norm(final_translation), 6))

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
