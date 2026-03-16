import unittest

from sailor_sdk.planner import (
    AbsolutePoseTOPPRAPlanner,
    RelativePoseTOPPRAPlanner,
    build_relative_distance_target,
    dq_pose_to_quaternion_translation,
)
from sailor_sdk.solver_client import RetimedTrajectory, SolveResult, SolverState, pose_dq_from_quaternion_translation


class FakeSolver:
    def __init__(self) -> None:
        self.solve_calls = []
        self.retime_calls = []

    def get_state(self, q):
        return SolverState(
            absolute_pose=[1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.8],
            relative_pose=[1.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, 0.0],
            left_pose=[1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.9],
            right_pose=[1.0, 0.0, 0.0, 0.0, 0.0, 0.3, -0.2, 0.7],
            joint_positions=[float(value) for value in q],
            joint_names=[f'q{i}' for i in range(len(q))],
        )

    def solve_absolute(self, *, q, target_pose, include_waist=False, keep_body_z_vertical=False, joint_limit_avoidance=True):
        self.solve_calls.append({
            'mode': 'absolute',
            'q': list(q),
            'target_pose': list(target_pose),
            'include_waist': include_waist,
            'keep_body_z_vertical': keep_body_z_vertical,
            'joint_limit_avoidance': joint_limit_avoidance,
        })
        next_q = [float(value) + 0.01 for value in q]
        return SolveResult(
            q=next_q,
            reason='converged',
            converged=True,
            apply_result=True,
            final_command_norm=0.0,
            final_invariant_norm=0.0,
        )

    def solve_relative(self, *, q, target_pose, include_waist=False, keep_body_z_vertical=False, joint_limit_avoidance=True):
        self.solve_calls.append({
            'mode': 'relative',
            'q': list(q),
            'target_pose': list(target_pose),
            'include_waist': include_waist,
            'keep_body_z_vertical': keep_body_z_vertical,
            'joint_limit_avoidance': joint_limit_avoidance,
        })
        next_q = [float(value) - 0.01 for value in q]
        return SolveResult(
            q=next_q,
            reason='converged',
            converged=True,
            apply_result=True,
            final_command_norm=0.0,
            final_invariant_norm=0.0,
        )

    def retime_waypoints(self, waypoints, max_vel=0.45, max_acc=0.9, gridpoints_hint=0, sample_count=0):
        self.retime_calls.append({
            'waypoints': [list(row) for row in waypoints],
            'max_vel': max_vel,
            'max_acc': max_acc,
            'gridpoints_hint': gridpoints_hint,
            'sample_count': sample_count,
        })
        return RetimedTrajectory(
            duration=1.2,
            waypoint_times=[0.0, 0.6, 1.2],
            sample_times=[0.0, 0.6, 1.2],
            positions=[list(waypoints[0]), list(waypoints[1]), list(waypoints[-1])],
            velocities=[[0.0] * len(waypoints[0])] * 3,
            accelerations=[[0.0] * len(waypoints[0])] * 3,
            gridpoints=[0.0, 1.0, 2.0],
            grid_times=[0.0, 0.6, 1.2],
            path_velocity_squared=[0.0, 1.0, 0.0],
        )


class AbsolutePoseTOPPRAPlannerTests(unittest.TestCase):
    def test_plan_absolute_pose_builds_joint_waypoints_and_retimes(self) -> None:
        planner = AbsolutePoseTOPPRAPlanner()
        solver = FakeSolver()
        q0 = [0.0] * 20
        target_pose = [1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1.0]

        plan = planner.plan_absolute_pose(
            solver,
            q_start=q0,
            target_pose=target_pose,
            include_waist=True,
            waypoint_count=3,
            sample_count=3,
        )

        self.assertEqual(len(plan.joint_waypoints), 3)
        self.assertEqual(len(plan.pose_waypoints), 3)
        self.assertEqual(len(solver.solve_calls), 2)
        self.assertEqual(solver.solve_calls[0]['q'], q0)
        self.assertTrue(solver.solve_calls[0]['include_waist'])
        self.assertEqual(solver.retime_calls[0]['sample_count'], 3)
        self.assertEqual(plan.trajectory.duration, 1.2)
        self.assertEqual(plan.final_state.joint_positions[0], 0.02)

    def test_plan_absolute_pose_passes_keep_body_z_vertical_to_solver(self) -> None:
        planner = AbsolutePoseTOPPRAPlanner()
        solver = FakeSolver()

        planner.plan_absolute_pose(
            solver,
            q_start=[0.0] * 20,
            target_pose=[1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1.0],
            include_waist=True,
            keep_body_z_vertical=True,
            waypoint_count=3,
            sample_count=3,
        )

        self.assertTrue(solver.solve_calls[0]['keep_body_z_vertical'])

    def test_plan_relative_pose_builds_joint_waypoints_and_retimes(self) -> None:
        planner = RelativePoseTOPPRAPlanner()
        solver = FakeSolver()

        plan = planner.plan_relative_pose(
            solver,
            q_start=[0.0] * 20,
            target_pose=[1.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0],
            include_waist=False,
            waypoint_count=3,
            sample_count=3,
        )

        self.assertEqual(len(plan.joint_waypoints), 3)
        self.assertEqual(solver.solve_calls[0]['mode'], 'relative')
        self.assertFalse(solver.solve_calls[0]['include_waist'])
        self.assertEqual(plan.trajectory.duration, 1.2)

    def test_build_relative_distance_target_reduces_translation_norm(self) -> None:
        current_pose = pose_dq_from_quaternion_translation(
            qw=1.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            x=0.3,
            y=0.4,
            z=0.0,
        )

        target_pose, current_distance, target_distance = build_relative_distance_target(current_pose, 0.2)
        _, target_translation = dq_pose_to_quaternion_translation(target_pose)

        self.assertAlmostEqual(current_distance, 0.5)
        self.assertAlmostEqual(target_distance, 0.3)
        self.assertAlmostEqual(target_translation[0], 0.18)
        self.assertAlmostEqual(target_translation[1], 0.24)


if __name__ == '__main__':
    unittest.main()
