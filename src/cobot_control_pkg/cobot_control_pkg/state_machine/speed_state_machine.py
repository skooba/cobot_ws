#!/usr/bin/env python3
# Standard Python imports
from enum import Enum

# ROS2 imports
import smach
from rclpy.node import Node
from example_interfaces.msg import String


class SpeedStateDataKeys(Enum):
    ESTOP = "estop"
    DISTANCE = "distance"


class SpeedStateOutcomes(Enum):
    ESTOP = "emergency_stop"
    STOPPED = "stopped"
    SLOW_SPEED = "slow"
    FULL_SPEED = "full"
    DONE = "done"


class SpeedControlVariables(Enum):
    MAX_SLOW_SPEED_DISTANCE = 800
    MIN_SLOW_SPEED_DISTANCE = 400


class FullSpeedState(smach.State):
    """
    This state is used when the robot is moving at full speed
    """

    def __init__(self, node: Node = None):
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.FULL_SPEED.value
            self.node.get_logger().info(f"Publishing speed state: {msg.data}")
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class SlowSpeedState(smach.State):
    """
    Action state that moves the robot at slow speed
    """

    def __init__(self, node: Node = None):
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.SLOW_SPEED.value
            self.node.get_logger().info(f"Publishing speed state: {msg.data}")
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class StoppedState(smach.State):
    """
    Action state that stops the robot
    """

    def __init__(self, node: Node = None):
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.STOPPED.value
            self.node.get_logger().info(f"Publishing speed state: {msg.data}")
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class EmergencyStopState(smach.State):
    """
    Action state that stops the robot
    """

    def __init__(self, node: Node = None):
        smach.State.__init__(self, outcomes=[SpeedStateOutcomes.DONE.value])
        self.node = node

    def execute(self, userdata) -> str:
        if self.node:
            # Publish the speed state
            msg = String()
            msg.data = SpeedStateOutcomes.ESTOP.value
            self.node.get_logger().info(f"Publishing speed state: {msg.data}")
            self.node.speed_state_publisher_.publish(msg)
        return SpeedStateOutcomes.DONE.value


class SpeedDecisionState(smach.State):
    """
    Top level decision state that switches between the three speed control
    states.
    """

    def __init__(self):
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
