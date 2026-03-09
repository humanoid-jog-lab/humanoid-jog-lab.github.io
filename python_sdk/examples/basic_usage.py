#!/usr/bin/env python3

import rclpy

from sailor_sdk import NullspaceSolverClient, SailorROS2Interface, pose_dq_from_quaternion_translation


def main() -> None:
    rclpy.init()

    ros = SailorROS2Interface()
    solver = NullspaceSolverClient()

    try:
        absolute_target = pose_dq_from_quaternion_translation(
            qw=1.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            x=0.45,
            y=0.0,
            z=1.05,
        )

        result = ros.solve_absolute_and_publish(
            solver,
            target_pose=absolute_target,
            include_waist=True,
        )
        print(result)
    finally:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
