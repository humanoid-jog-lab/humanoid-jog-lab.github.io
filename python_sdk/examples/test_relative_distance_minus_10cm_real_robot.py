#!/usr/bin/env python3

import math

import rclpy

from sailor_sdk import (
    NullspaceSolverClient,
    RelativePoseTOPPRAPlanner,
    SailorROS2Interface,
    build_relative_distance_target,
    dq_pose_to_quaternion_translation,
)


RELATIVE_DISTANCE_DELTA = 0.05
WAYPOINT_COUNT = 12
SAMPLE_COUNT = 120
JOINT_STATE_TIMEOUT_SECONDS = 5.0


def translation_norm(translation) -> float:
    return math.sqrt(sum(float(component) * float(component) for component in translation))


def main() -> None:
    rclpy.init()

    ros = SailorROS2Interface()
    solver = NullspaceSolverClient()
    planner = RelativePoseTOPPRAPlanner()

    try:
        current_q = ros.wait_for_joint_state(timeout_sec=JOINT_STATE_TIMEOUT_SECONDS)
        start_state = solver.get_state(current_q)
        target_pose, current_distance, target_distance = build_relative_distance_target(
            start_state.relative_pose,
            RELATIVE_DISTANCE_DELTA,
        )
        _, start_translation = dq_pose_to_quaternion_translation(start_state.relative_pose)
        _, target_translation = dq_pose_to_quaternion_translation(target_pose)

        print('received current joint state from /whole/joint_states')
        print('start relative translation =', [round(value, 6) for value in start_translation])
        print('target relative translation =', [round(value, 6) for value in target_translation])
        print('current relative distance =', round(current_distance, 6))
        print('target relative distance =', round(target_distance, 6))
        print('distance delta =', RELATIVE_DISTANCE_DELTA)

        plan = planner.plan_relative_pose(
            solver,
            q_start=current_q,
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

        print('publishing real arm commands only')
        ros.execute_timed_trajectory(
            plan.trajectory,
            publish_waist=False,
        )
    finally:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
