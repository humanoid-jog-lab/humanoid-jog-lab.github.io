import unittest
from unittest import mock

from sailor_sdk.messages import RobotArmServoCommand, RobotServoCommand, build_joint_state_payload
from sailor_sdk.solver_client import NullspaceSolverClient, pose_dq_from_quaternion_translation


class SolverClientTests(unittest.TestCase):
    def test_pose_dq_from_quaternion_translation_returns_eight_numbers(self) -> None:
        dq = pose_dq_from_quaternion_translation(
            qw=1.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            x=0.1,
            y=0.2,
            z=0.3,
        )

        self.assertEqual(len(dq), 8)
        self.assertEqual(dq[:4], [1.0, 0.0, 0.0, 0.0])

    def test_robot_arm_command_uses_humanoid_arm_slices(self) -> None:
        q = list(range(20))
        payload = RobotArmServoCommand.from_q(q).as_dict()

        self.assertEqual(payload['ljoint1_position'], 4.0)
        self.assertEqual(payload['ljoint7_position'], 10.0)
        self.assertEqual(payload['rjoint1_position'], 11.0)
        self.assertEqual(payload['rjoint7_position'], 17.0)

    def test_robot_servo_command_defaults_vel_to_point_three(self) -> None:
        q = list(range(20))
        payload = RobotServoCommand.from_q(q).as_dict()

        self.assertEqual(payload['ankle_position'], 0.0)
        self.assertEqual(payload['hip_yaw_position'], 3.0)
        self.assertEqual(payload['vel'], 0.3)

    def test_joint_state_payload_uses_humanoid_joint_order(self) -> None:
        q = list(range(20))
        payload = build_joint_state_payload(q)

        self.assertEqual(payload['name'][0], 'ankle_joint')
        self.assertEqual(payload['name'][-1], 'head_pitch_joint')
        self.assertEqual(payload['position'][4], 4.0)
        self.assertEqual(payload['position'][17], 17.0)
        self.assertEqual(payload['velocity'], [0.0] * 20)

    @mock.patch('sailor_sdk.solver_client.subprocess.run')
    def test_solver_client_parses_solver_output(self, run_mock: mock.Mock) -> None:
        run_mock.return_value = mock.Mock(
            returncode=0,
            stdout='\n'.join([
                'status=ok',
                'q=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19',
                'reason=practical_converged',
                'converged=true',
                'apply_result=true',
                'final_command_norm=0.001',
                'final_invariant_norm=0.002',
            ]),
            stderr='',
        )

        client = NullspaceSolverClient(solver_binary='/tmp/fake_solver')
        result = client.solve_absolute(
            q=list(range(20)),
            target_pose=[0.0] * 8,
        )

        self.assertTrue(result.converged)
        self.assertTrue(result.apply_result)
        self.assertEqual(result.reason, 'practical_converged')
        self.assertEqual(result.q[17], 17.0)

    @mock.patch('sailor_sdk.solver_client.subprocess.run')
    def test_solver_client_passes_keep_body_z_vertical_flag(self, run_mock: mock.Mock) -> None:
        run_mock.return_value = mock.Mock(
            returncode=0,
            stdout='\n'.join([
                'status=ok',
                'q=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19',
                'reason=converged',
                'converged=true',
                'apply_result=true',
                'final_command_norm=0.0',
                'final_invariant_norm=0.0',
            ]),
            stderr='',
        )

        client = NullspaceSolverClient(solver_binary='/tmp/fake_solver')
        client.solve_absolute(
            q=list(range(20)),
            target_pose=[0.0] * 8,
            include_waist=True,
            keep_body_z_vertical=True,
        )

        command = run_mock.call_args.kwargs['args'] if 'args' in run_mock.call_args.kwargs else run_mock.call_args.args[0]
        flag_index = command.index('--keep-body-z-vertical')
        self.assertEqual(command[flag_index + 1], '1')

    @mock.patch('sailor_sdk.solver_client.subprocess.run')
    def test_solver_client_parses_state_output_with_joint_metadata(self, run_mock: mock.Mock) -> None:
        run_mock.return_value = mock.Mock(
            returncode=0,
            stdout='\n'.join([
                'status=ok',
                'absolute_pose=1,0,0,0,0,1,2,3',
                'relative_pose=1,0,0,0,0,4,5,6',
                'left_pose=1,0,0,0,0,7,8,9',
                'right_pose=1,0,0,0,0,10,11,12',
            ]),
            stderr='',
        )

        q = list(range(20))
        client = NullspaceSolverClient(solver_binary='/tmp/fake_solver')
        state = client.get_state(q)

        self.assertEqual(state.joint_positions, [float(value) for value in q])
        self.assertEqual(len(state.joint_names), 20)
        self.assertEqual(state.absolute_pose[-3:], [1.0, 2.0, 3.0])
        self.assertEqual(state.relative_pose[-3:], [4.0, 5.0, 6.0])

    @mock.patch('sailor_sdk.solver_client.subprocess.run')
    def test_solver_client_parses_retimed_trajectory_output(self, run_mock: mock.Mock) -> None:
        run_mock.return_value = mock.Mock(
            returncode=0,
            stdout='\n'.join([
                'status=ok',
                'duration=1.5',
                'waypoint_times=0,0.7,1.5',
                'sample_times=0,0.5,1.0,1.5',
                'positions=0,1;2,3;4,5;6,7',
                'velocities=0,0;0.1,0.2;0.1,0.2;0,0',
                'accelerations=0.3,0.4;0,0;0,0;-0.3,-0.4',
                'gridpoints=0,1,2',
                'grid_times=0,0.7,1.5',
                'path_velocity_squared=0,1,0',
            ]),
            stderr='',
        )

        client = NullspaceSolverClient(solver_binary='/tmp/fake_solver')
        trajectory = client.retime_waypoints(
            [[0, 1], [2, 3], [4, 5]],
            max_vel=0.45,
            max_acc=0.9,
            sample_count=4,
        )

        self.assertEqual(trajectory.duration, 1.5)
        self.assertEqual(trajectory.sample_times, [0.0, 0.5, 1.0, 1.5])
        self.assertEqual(trajectory.positions[2], [4.0, 5.0])
        self.assertEqual(trajectory.velocities[1], [0.1, 0.2])
        self.assertEqual(trajectory.accelerations[-1], [-0.3, -0.4])


if __name__ == '__main__':
    unittest.main()
