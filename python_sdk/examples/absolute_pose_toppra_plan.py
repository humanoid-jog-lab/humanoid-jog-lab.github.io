#!/usr/bin/env python3

import rclpy

from sailor_sdk import (
    AbsolutePoseTOPPRAPlanner,
    NullspaceSolverClient,
    SailorROS2Interface,
)


def main() -> None:
    rclpy.init()

    ros = SailorROS2Interface()
    solver = NullspaceSolverClient()
    planner = AbsolutePoseTOPPRAPlanner()

    try:
        state = ros.get_current_state(solver)
        target_pose = state.absolute_pose[:]
        target_pose[5] += 0.05

        plan = planner.plan_absolute_pose(
            solver,
            q_start=ros.current_q,
            target_pose=target_pose,
            include_waist=True,
            waypoint_count=8,
            sample_count=80,
        )

        print(f'planned duration: {plan.trajectory.duration:.3f}s')
        print(f'joint waypoints: {len(plan.joint_waypoints)}')
        ros.execute_timed_trajectory(plan.trajectory)
    finally:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
