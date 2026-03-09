from __future__ import annotations

import time
from typing import Callable, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_control_msg.msg import Robotarmservomsg, Robotservomsg

from .constants import (
    DEFAULT_ARM_COMMAND_TOPIC,
    DEFAULT_INITIAL_Q,
    DEFAULT_JOINT_STATE_TOPIC,
    DEFAULT_WAIST_COMMAND_TOPIC,
    HUMANOID_JOINT_NAMES,
    WHEEL_JOINT_NAMES,
)
from .messages import RobotArmServoCommand, RobotServoCommand, build_joint_state_payload
from .solver_client import NullspaceSolverClient, SolveResult


class SailorROS2Interface(Node):
    def __init__(
        self,
        *,
        node_name: str = 'sailor_python_sdk',
        joint_state_topic: str = DEFAULT_JOINT_STATE_TOPIC,
        joint_state_publish_topic: str = DEFAULT_JOINT_STATE_TOPIC,
        arm_command_topic: str = DEFAULT_ARM_COMMAND_TOPIC,
        waist_command_topic: str = DEFAULT_WAIST_COMMAND_TOPIC,
        initial_q: Sequence[float] | None = None,
    ) -> None:
        super().__init__(node_name)
        self.current_q: List[float] = [float(value) for value in (initial_q or DEFAULT_INITIAL_Q)]
        self.has_received_joint_state = False
        self._joint_index = {name: index for index, name in enumerate(HUMANOID_JOINT_NAMES)}
        self._callbacks: List[Callable[[List[float]], None]] = []

        self._joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self._on_joint_state,
            10,
        )
        self._joint_state_publisher = self.create_publisher(JointState, joint_state_publish_topic, 10)
        self._arm_publisher = self.create_publisher(Robotarmservomsg, arm_command_topic, 10)
        self._waist_publisher = self.create_publisher(Robotservomsg, waist_command_topic, 10)

    def add_joint_state_callback(self, callback: Callable[[List[float]], None]) -> None:
        self._callbacks.append(callback)

    def get_current_state(self, solver: NullspaceSolverClient):
        return solver.get_state(self.current_q)

    def get_current_absolute_pose(self, solver: NullspaceSolverClient) -> List[float]:
        return self.get_current_state(solver).absolute_pose

    def get_current_relative_pose(self, solver: NullspaceSolverClient) -> List[float]:
        return self.get_current_state(solver).relative_pose

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else []
        positions = list(msg.position) if msg.position else []
        count = min(len(names), len(positions))

        for index in range(count):
            name = str(names[index])
            if name in WHEEL_JOINT_NAMES:
                continue
            q_index = self._joint_index.get(name)
            if q_index is None:
                continue
            self.current_q[q_index] = float(positions[index])

        self.has_received_joint_state = True
        for callback in self._callbacks:
            callback(self.current_q[:])

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> List[float]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if self.has_received_joint_state:
                return self.current_q[:]
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        raise TimeoutError(f'No JointState received within {timeout_sec:.2f}s on subscribed topic')

    def publish_arm_command(
        self,
        q: Sequence[float],
        *,
        run_mode: int = 1,
        robot_power: bool = True,
        run_time: float = 0.0,
    ) -> Robotarmservomsg:
        command = RobotArmServoCommand.from_q(
            q=q,
            run_mode=run_mode,
            robot_power=robot_power,
            run_time=run_time,
        )
        payload = command.as_dict()
        message = Robotarmservomsg()
        for key, value in payload.items():
            setattr(message, key, value)
        message.stamp = self.get_clock().now().to_msg()
        self._arm_publisher.publish(message)
        return message

    def publish_joint_state(
        self,
        q: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        effort: Sequence[float] | None = None,
        frame_id: str = 'base_link',
    ) -> JointState:
        payload = build_joint_state_payload(q, velocities=velocities, effort=effort)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.name = payload['name']
        message.position = payload['position']
        message.velocity = payload['velocity']
        message.effort = payload['effort']
        self.current_q = [float(value) for value in payload['position']]
        self._joint_state_publisher.publish(message)
        return message

    def publish_waist_command(
        self,
        q: Sequence[float],
        *,
        run_mode: int = 0,
        x: float = 0.0,
        y: float = 0.0,
        phi: float = 0.0,
        mode_leg_select: int = 0,
        vel: float = 0.3,
    ) -> Robotservomsg:
        command = RobotServoCommand.from_q(
            q=q,
            run_mode=run_mode,
            x=x,
            y=y,
            phi=phi,
            mode_leg_select=mode_leg_select,
            vel=vel,
        )
        payload = command.as_dict()
        message = Robotservomsg()
        for key, value in payload.items():
            setattr(message, key, value)
        self._waist_publisher.publish(message)
        return message

    def solve_absolute_and_publish(
        self,
        solver: NullspaceSolverClient,
        *,
        target_pose: Sequence[float],
        include_waist: bool = False,
        keep_body_z_vertical: bool = False,
        joint_limit_avoidance: bool = True,
        waist_vel: float = 0.3,
    ) -> SolveResult:
        result = solver.solve_absolute(
            q=self.current_q,
            target_pose=target_pose,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
        )
        if result.apply_result:
            self.current_q = result.q[:]
            self.publish_arm_command(result.q)
            self.publish_waist_command(result.q, vel=waist_vel)
        return result

    def execute_timed_trajectory(
        self,
        trajectory,
        *,
        robot_power: bool = True,
        arm_run_time: float = 0.0,
        waist_vel: float = 0.3,
        publish_waist: bool = True,
    ) -> None:
        start_time = time.monotonic()
        for index, sample_time in enumerate(trajectory.sample_times):
            now = time.monotonic()
            sleep_duration = sample_time - (now - start_time)
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            q = trajectory.positions[index]
            self.current_q = [float(value) for value in q]
            self.publish_arm_command(
                q,
                robot_power=robot_power,
                run_time=arm_run_time,
            )
            if publish_waist:
                self.publish_waist_command(q, vel=waist_vel)

    def execute_timed_joint_state_trajectory(
        self,
        trajectory,
        *,
        frame_id: str = 'base_link',
    ) -> None:
        start_time = time.monotonic()
        for index, sample_time in enumerate(trajectory.sample_times):
            now = time.monotonic()
            sleep_duration = sample_time - (now - start_time)
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            q = trajectory.positions[index]
            velocities = self._estimate_joint_state_velocity(trajectory, index)
            self.publish_joint_state(q, velocities=velocities, frame_id=frame_id)

    def solve_relative_and_publish(
        self,
        solver: NullspaceSolverClient,
        *,
        target_pose: Sequence[float],
        include_waist: bool = False,
        keep_body_z_vertical: bool = False,
        joint_limit_avoidance: bool = True,
        waist_vel: float = 0.3,
    ) -> SolveResult:
        result = solver.solve_relative(
            q=self.current_q,
            target_pose=target_pose,
            include_waist=include_waist,
            keep_body_z_vertical=keep_body_z_vertical,
            joint_limit_avoidance=joint_limit_avoidance,
        )
        if result.apply_result:
            self.current_q = result.q[:]
            self.publish_arm_command(result.q)
            self.publish_waist_command(result.q, vel=waist_vel)
        return result

    def _estimate_joint_state_velocity(self, trajectory, index: int) -> List[float]:
        if index <= 0 or index >= len(trajectory.positions):
            return [0.0] * len(HUMANOID_JOINT_NAMES)

        previous_time = float(trajectory.sample_times[index - 1])
        current_time = float(trajectory.sample_times[index])
        dt = current_time - previous_time
        if dt <= 1e-9:
            return [0.0] * len(HUMANOID_JOINT_NAMES)

        previous_q = trajectory.positions[index - 1]
        current_q = trajectory.positions[index]
        return [
            (float(current_q[joint_index]) - float(previous_q[joint_index])) / dt
            for joint_index in range(len(HUMANOID_JOINT_NAMES))
        ]
