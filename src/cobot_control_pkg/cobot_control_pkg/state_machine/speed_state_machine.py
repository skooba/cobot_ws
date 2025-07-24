#!/usr/bin/env python3
"""State machine components for robot speed control based on sensor data."""
# Standard Python imports
from enum import Enum

# 3rd party imports
from example_interfaces.msg import String
from rclpy.node import Node
import smach


class SpeedStateDataKeys(Enum):
    """Data keys used in the speed control state machine."""

    ESTOP = 'estop'
    DISTANCE = 'distance'


class SpeedStateOutcomes(Enum):
    """Possible outcomes from speed control states."""

    ESTOP = 'emergency_stop'
    STOPPED = 'stopped'
    SLOW_SPEED = 'slow'
    FULL_SPEED = 'full'
    DONE = 'done'


class SpeedControlVariables(Enum):
    """Configuration variables for speed control thresholds."""

    MAX_SLOW_SPEED_DISTANCE = 800
    MIN_SLOW_SPEED_DISTANCE = 400


class FullSpeedState(smach.State):
    """This state is used when the robot is moving at full speed."""

    def __init__(self, node: Node = None):
        """Initialize the full speed state."""
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        """Execute full speed state and publish speed command."""
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.FULL_SPEED.value
            self.node.get_logger().info(f'Publishing speed state: {msg.data}')
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class SlowSpeedState(smach.State):
    """Action state that moves the robot at slow speed."""

    def __init__(self, node: Node = None):
        """Initialize the slow speed state."""
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        """Execute slow speed state and publish speed command."""
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.SLOW_SPEED.value
            self.node.get_logger().info(f'Publishing speed state: {msg.data}')
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class StoppedState(smach.State):
    """Action state that stops the robot."""

    def __init__(self, node: Node = None):
        """Initialize the stopped state."""
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        """Execute stopped state and publish stop command."""
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.STOPPED.value
            self.node.get_logger().info(f'Publishing speed state: {msg.data}')
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class EmergencyStopState(smach.State):
    """Action state that stops the robot."""

    def __init__(self, node: Node = None):
        """Initialize the emergency stop state."""
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        """Execute emergency stop state and publish emergency stop command."""
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.ESTOP.value
            self.node.get_logger().info(f'Publishing speed state: {msg.data}')
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class SpeedDecisionState(smach.State):
    """
    Top level decision state that switches between the three speed control states.

    This state evaluates sensor data and determines the appropriate speed mode.
    """

    def __init__(self):
        """Initialize the speed decision state."""
        smach.State.__init__(
            self,
            outcomes=[
                SpeedStateOutcomes.STOPPED.value,
                SpeedStateOutcomes.SLOW_SPEED.value,
                SpeedStateOutcomes.FULL_SPEED.value,
                SpeedStateOutcomes.ESTOP.value,
            ],
            input_keys=[
                SpeedStateDataKeys.ESTOP.value,
                SpeedStateDataKeys.DISTANCE.value,
            ],
        )

    def execute(self, userdata) -> str:
        """Execute speed decision logic based on sensor data."""
        estop = getattr(userdata, SpeedStateDataKeys.ESTOP.value)
        distance = getattr(userdata, SpeedStateDataKeys.DISTANCE.value)
        if estop:
            return SpeedStateOutcomes.ESTOP.value
        elif distance < SpeedControlVariables.MIN_SLOW_SPEED_DISTANCE.value:
            return SpeedStateOutcomes.STOPPED.value
        elif distance <= SpeedControlVariables.MAX_SLOW_SPEED_DISTANCE.value:
            return SpeedStateOutcomes.SLOW_SPEED.value
        else:
            return SpeedStateOutcomes.FULL_SPEED.value
