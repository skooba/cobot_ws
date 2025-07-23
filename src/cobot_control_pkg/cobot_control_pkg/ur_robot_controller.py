#!/usr/bin/env python3
# internal imports
from math import ceil
from cobot_control_pkg.state_machine.speed_state_machine import (
    SpeedStateOutcomes,
)

# Python imports
from enum import Enum
import threading
import time

# Ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from example_interfaces.msg import String


class VelocityValues(Enum):
    SLOW_SPEED = 0.3
    FULL_SPEED = 1.0


class URRobotController(Node):
    def __init__(self):
        super().__init__("ur_robot_controller")

        # Parameters for the velocity ramping
        # ramp_rate: float - the acceleration rate during ramps (per second)
        # frequency: float - the frequency that velocity updates are sent (Hz)
        self.declare_parameter("ramp_rate", 0.5)
        self.declare_parameter("frequency", 10.0)

        self.ramp_rate = self.get_parameter("ramp_rate").value
        self.frequency = self.get_parameter("frequency").value

        # UR Robot joint names in order (indices 0-5 in Float64MultiArray)
        self.joint_names = [
            "shoulder_pan_joint",  # index 0
            "shoulder_lift_joint",  # index 1
            "elbow_joint",  # index 2
            "wrist_1_joint",  # index 3
            "wrist_2_joint",  # index 4
            "wrist_3_joint",  # index 5
        ]

        # Create the publisher for the velocity commands to be published on
        # Topic: /forward_velocity_controller/commands
        # Message Type: std_msgs/Float64MultiArray
        self.velocity_publisher_ = self.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 10
        )

        # Create subscriber for speed state messages
        # Topic: /robot_speed_state
        # Message Type: example_interfaces/String
        self.speed_state_subscriber = self.create_subscription(
            String, "robot_speed_state", self.speed_state_callback, 10
        )

        # Thread management for velocity commands
        self._ramp_thread: threading.Thread = None
        self._thread_lock: threading.Lock = threading.Lock()
        self._stop_ramp_event: threading.Event = threading.Event()

        # State Tracking
        self.current_speed_state = SpeedStateOutcomes.STOPPED.value
        self.estop_active = False
        self.current_velocities = [0.0] * 6

    def speed_state_callback(self, msg: String) -> None:
        """
        Callback function for control logic
        Tells the robot controller what to do based on the speed state
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
                [round(VelocityValues.SLOW_SPEED.value, 3)] * 6
            )
        elif self.current_speed_state == SpeedStateOutcomes.FULL_SPEED.value:
            # Full speed state will only be sent if the estop is not active
            if self.estop_active:
                self.estop_active = False
            self.send_ramped_velocity_command(
                [round(VelocityValues.FULL_SPEED.value, 3)] * 6
            )
        elif self.current_speed_state == SpeedStateOutcomes.ESTOP.value:
            # Estop will stop the robot immediately
            self.estop_robot()

    def send_ramped_velocity_command(
        self, target_velocities: list[float]
    ) -> None:
        """
        Send a velocity command to the robot with a ramp rate
        args:
            target_velocities: list[float] - the target velocity values
        """
        target_velocities = [round(v, 3) for v in target_velocities]

        if len(target_velocities) != 6:
            self.get_logger().error(
                f"Need exactly 6 velocity values, got {len(target_velocities)}"
            )
            return

        with self._thread_lock:
            # Stop any existing ramp thread
            if self._ramp_thread is not None and self._ramp_thread.is_alive():
                self._stop_ramp_event.set()
                self._ramp_thread.join()

            # Reset the stop event and start new thread
            self._stop_ramp_event.clear()
            self._ramp_thread = threading.Thread(
                target=self._ramp_worker,
                args=(target_velocities, self.ramp_rate, self.frequency),
                daemon=True,
            )
            self._ramp_thread.start()

    def _ramp_worker(
        self,
        target_velocities: list[float],
        ramp_rate: float,
        frequency: float,
    ) -> None:
        """
        Worker thread for ramping velocity commands
        """

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

        while not self._stop_ramp_event.is_set() and update_times:
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
                with self._thread_lock:
                    if self.estop_active:
                        return
                    self.send_immediate_velocity_command(next_velocities)
                self.current_velocities = next_velocities.copy()

                # remove the update time that has been processed from the list
                update_times.pop(0)
        self.get_logger().info(f"velocities: {self.current_velocities}")

    def send_immediate_velocity_command(self, velocities: list[float]) -> None:
        """
        Send a velocity command to the robot
        """
        if len(velocities) != 6:
            self.get_logger().error(
                f"Need exactly 6 velocity values, got {len(velocities)}"
            )
            return
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_publisher_.publish(msg)

    def estop_robot(self):
        """Stop all robot motion"""
        # lock the critical resource of estop_active turning true
        with self._thread_lock:
            self.send_immediate_velocity_command([0.0] * 6)
            self.current_velocities = [0.0] * 6
            self.estop_active = True


def main(args=None):
    rclpy.init(args=args)
    node = URRobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
