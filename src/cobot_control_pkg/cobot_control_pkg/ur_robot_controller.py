#!/usr/bin/env python3
"""UR robot controller node for velocity command management."""
# Python imports
from enum import Enum
from math import ceil
import threading
import time

# Third Party imports
from cobot_control_pkg.state_machine.speed_state_machine import (
    SpeedStateOutcomes,
)
from example_interfaces.msg import String
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray


class VelocityValues(Enum):
    """Default velocity values for robot speed control."""

    SLOW_SPEED = 0.3
    FULL_SPEED = 1.0


class URRobotController(Node):
    """ROS2 node for controlling UR robot velocity with ramping and safety features."""

    def __init__(self):
        """Initialize the UR robot controller with parameters and publishers."""
        super().__init__('ur_robot_controller')

        # Parameters for the velocity ramping
        # ramp_rate: float - the acceleration rate during ramps (per second)
        # frequency: float - the frequency that velocity updates are sent (Hz)
        # slow_speed: float - the speed to move at when the object is close to the robot
        # full_speed: float - the speed to move at when the object is far from the robot
        self.declare_parameter('ramp_rate', 0.5)
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('slow_speed', 0.3)
        self.declare_parameter('full_speed', 1.0)

        self.ramp_rate = self.get_parameter('ramp_rate').value
        self.frequency = self.get_parameter('frequency').value
        self.slow_speed = self.get_parameter('slow_speed').value
        self.full_speed = self.get_parameter('full_speed').value

        self.add_on_set_parameters_callback(self.ur_robot_parameters_callback)

        # UR Robot joint names in order (indices 0-5 in Float64MultiArray)
        self.joint_names = [
            'shoulder_pan_joint',  # index 0
            'shoulder_lift_joint',  # index 1
            'elbow_joint',  # index 2
            'wrist_1_joint',  # index 3
            'wrist_2_joint',  # index 4
            'wrist_3_joint',  # index 5
        ]

        # Create the publisher for the velocity commands to be published on
        # Topic: /forward_velocity_controller/commands
        # Message Type: std_msgs/Float64MultiArray
        self.velocity_publisher_ = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )

        # Create subscriber for speed state messages
        # Topic: /robot_speed_state
        # Message Type: example_interfaces/String
        self.speed_state_subscriber = self.create_subscription(
            String, 'robot_speed_state', self.speed_state_callback, 10
        )

        # Thread management for velocity commands
        self.ramp_thread: threading.Thread = None
        self.ramp_thread_lock: threading.Lock = threading.Lock()
        self.stop_ramp_event: threading.Event = threading.Event()

        # State Tracking
        self.current_speed_state = SpeedStateOutcomes.STOPPED.value
        self.estop_active = False
        self.current_velocities = [0.0] * 6

    def ur_robot_parameters_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Handle parameter changes during runtime."""
        result = False
        for param in params:
            if param.name == 'slow_speed':
                if param.value > 0 and param.value <= self.full_speed:
                    self.slow_speed = param.value
                    self.get_logger().info(f'Updated slow_speed to {param.value}')
                    result = True
                else:
                    self.get_logger().error(f'Invalid slow_speed value: {param.value}')
                    result = False
            if param.name == 'full_speed':
                if param.value >= self.slow_speed:
                    self.full_speed = param.value
                    self.get_logger().info(f'Updated full_speed to {param.value}')
                    result = True
                else:
                    self.get_logger().error(f'Invalid full_speed value: {param.value}')
                    result = False
            if param.name == 'ramp_rate':
                if param.value > 0:
                    self.ramp_rate = param.value
                    self.get_logger().info(f'Updated ramp_rate to {param.value}')
                    result = True
                else:
                    self.get_logger().error(f'Invalid ramp_rate value: {param.value}')
                    result = False
            if param.name == 'frequency':
                if param.value > 0:
                    self.frequency = param.value
                    self.get_logger().info(f'Updated frequency to {param.value}')
                    result = True
                else:
                    self.get_logger().error(f'Invalid frequency value: {param.value}')
                    result = False
        return SetParametersResult(successful=result)

    def speed_state_callback(self, msg: String) -> None:
        """
        Handle control logic for robot based on speed state.

        Tells the robot controller what to do based on the speed state.
        """
        self.current_speed_state = msg.data

        # Logic to adjust robot behavior based on speed state
        # For example:
        if self.current_speed_state == SpeedStateOutcomes.STOPPED.value:
            # Stopped speed state will only be sent if the estop is not active
            if self.estop_active:
                self.estop_active = False
            # Stopped state will be sent if object is close to the robot or
            # after the estop is triggered and hasn't been reset
            self.send_ramped_velocity_command([0.0] * 6)
        elif self.current_speed_state == SpeedStateOutcomes.SLOW_SPEED.value:
            # Slow speed state will only be sent if the estop is not active
            if self.estop_active:
                self.estop_active = False
            self.send_ramped_velocity_command(
                [round(self.slow_speed, 3)] * 6
            )
        elif self.current_speed_state == SpeedStateOutcomes.FULL_SPEED.value:
            # Full speed state will only be sent if the estop is not active
            if self.estop_active:
                self.estop_active = False
            self.send_ramped_velocity_command(
                [round(self.full_speed, 3)] * 6
            )
        elif self.current_speed_state == SpeedStateOutcomes.ESTOP.value:
            # Estop will stop the robot immediately
            self.estop_robot()

    def send_ramped_velocity_command(
        self, target_velocities: list[float]
    ) -> None:
        """
        Send a velocity command to the robot with a ramp rate.

        Args:
        ----
        target_velocities: list[float] - the target velocity values

        """
        target_velocities = [round(v, 3) for v in target_velocities]

        if len(target_velocities) != 6:
            self.get_logger().error(
                f'Need exactly 6 velocity values, got {len(target_velocities)}'
            )
            return

        with self.ramp_thread_lock:
            # Stop any existing ramp thread
            if self.ramp_thread is not None and self.ramp_thread.is_alive():
                self.stop_ramp_event.set()
                self.ramp_thread.join()

            # Reset the stop event and start new thread
            self.stop_ramp_event.clear()
            self.ramp_thread = threading.Thread(
                target=self._ramp_worker,
                args=(target_velocities, self.ramp_rate, self.frequency),
                daemon=False,
            )
            self.ramp_thread.start()

    def _ramp_worker(
        self,
        target_velocities: list[float],
        ramp_rate: float,
        frequency: float,
    ) -> None:
        """Worker thread for ramping velocity commands."""
        dt = 1.0 / frequency  # Time step in seconds
        max_change_per_step = (
            ramp_rate * dt
        )  # Max velocity change per time step

        # Calculate number of updates needed
        number_updates = ceil(
            max(
                [
                    abs(a - b)
                    for a, b in zip(target_velocities, self.current_velocities)
                ]
            )
            / max_change_per_step
        )

        # Calculate the times at which the updates will be sent
        start_time = time.time()
        update_times = [
            start_time + i / frequency for i in range(number_updates)
        ]

        # Empty list to store the next velocities
        next_velocities = [0.0] * 6

        while not self.stop_ramp_event.is_set() and update_times:
            current_time = time.time()
            if current_time >= update_times[0]:
                # increment next velocities towards the target velocities
                for i in range(6):
                    diff = round(
                        target_velocities[i] - self.current_velocities[i], 3
                    )
                    if diff > max_change_per_step:
                        next_velocities[i] = (
                            self.current_velocities[i] + max_change_per_step
                        )
                    elif diff < -max_change_per_step:
                        next_velocities[i] = (
                            self.current_velocities[i] - max_change_per_step
                        )
                    else:
                        # the next velocity is the target velocity
                        next_velocities[i] = target_velocities[i]

                # lock the thread to prevent race condition where the estop
                # is triggered between the check for estop and the
                # publishing of the velocity command
                with self.ramp_thread_lock:
                    if self.estop_active:
                        return
                    self.send_immediate_velocity_command(next_velocities)
                self.current_velocities = next_velocities.copy()

                # remove the update time that has been processed from the list
                update_times.pop(0)
        self.get_logger().info(f'velocities: {self.current_velocities}')

    def send_immediate_velocity_command(self, velocities: list[float]) -> None:
        """Send a velocity command to the robot."""
        if len(velocities) != 6:
            self.get_logger().error(
                f'Need exactly 6 velocity values, got {len(velocities)}'
            )
            return
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_publisher_.publish(msg)

    def estop_robot(self) -> None:
        """Stop all robot motion."""
        # lock the critical resource of estop_active turning true
        with self.ramp_thread_lock:
            self.send_immediate_velocity_command([0.0] * 6)
            self.current_velocities = [0.0] * 6
            self.estop_active = True


def main(args=None):
    """Run the UR robot controller node."""
    rclpy.init(args=args)
    node = URRobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
