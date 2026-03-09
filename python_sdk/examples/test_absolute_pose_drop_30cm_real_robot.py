#!/usr/bin/env python3

import rclpy

from sailor_sdk import (
    AbsolutePoseTOPPRAPlanner,
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
JOINT_STATE_TIMEOUT_SECONDS = 5.0
WAIST_VEL = 0.3


def build_target_pose_from_current_state(state):
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
    return target_pose, translation


def main() -> None:
    rclpy.init()

    ros = SailorROS2Interface()
    solver = NullspaceSolverClient()
    planner = AbsolutePoseTOPPRAPlanner()

    try:
        current_q = ros.wait_for_joint_state(timeout_sec=JOINT_STATE_TIMEOUT_SECONDS)
        start_state = solver.get_state(current_q)
        target_pose, target_translation = build_target_pose_from_current_state(start_state)
        _, start_translation = dq_pose_to_quaternion_translation(start_state.absolute_pose)

        print('received current joint state from /whole/joint_states')
        print('start absolute translation =', [round(value, 6) for value in start_translation])
        print('target absolute translation =', [round(value, 6) for value in target_translation])
        print('drop distance z =', DROP_DISTANCE_Z)
        print('include waist =', INCLUDE_WAIST)
        print('keep body z vertical =', KEEP_BODY_Z_VERTICAL)

        plan = planner.plan_absolute_pose(
            solver,
            q_start=current_q,
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

        print('publishing real waist and arm commands')
        ros.execute_timed_trajectory(
            plan.trajectory,
            waist_vel=WAIST_VEL,
            publish_waist=INCLUDE_WAIST,
        )
    finally:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
