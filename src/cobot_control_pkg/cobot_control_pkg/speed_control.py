#!/usr/bin/env python3
# internal imports
from cobot_control_pkg.state_machine.speed_state_machine import (
    SpeedDecisionState, SpeedStateOutcomes, StoppedState,
    FullSpeedState, SlowSpeedState, SpeedStateDataKeys, 
    EmergencyStopState
)

# Python imports
import threading
from enum import Enum

# ROS2 imports
import rclpy
from rclpy.node import Node
import smach
import smach_ros
from example_interfaces.msg import Float32, Bool, String


class TopLevelOutcome(Enum):
    ABORTED = 'aborted'
    SENSOR_DATA_UPDATE = 'sensor_data_update'


class TopLevelState(Enum):
    MONITOR_PROXIMITY = 'monitor_prox'
    MONITOR_ESTOP = 'monitor_estop'
    CONCURRENT_MONITORING = 'conc_monitor'
    STOPPED_STATE = 'stopped_state'
    SLOW_SPEED_STATE = 'slow_speed_state'
    FULL_SPEED_STATE = 'full_speed_state'
    EMERGENCY_STOP_STATE = 'emergency_stop_state'
    SPEED_DECISION = 'speed_decision'


class SpeedControlNode(Node):
    def __init__(self):
        super().__init__("speed_controller")

        # Suppress SMACH logging
        smach_logger = rclpy.logging.get_logger('smach_ros')
        smach_logger.set_level(rclpy.logging.LoggingSeverity.WARN)

        # Create publisher for speed state messages
        # Topic: /robot_speed_state
        # Message Type: example_interfaces/String
        self.speed_state_publisher_ = self.create_publisher(
            String,
            "/robot_speed_state",
            10
        )

        # Create a fsm with a dummy outcome since we will never exit the fsm
        self.fsm: smach.StateMachine = smach.StateMachine(
            outcomes=[TopLevelOutcome.ABORTED.value]
            )

        with self.fsm:

            # Create the sub concurrent monitoring state
            # No outcome map needed since the outcome will always be
            # SENSOR_DATA_UPDATE default
            monitor_concurrence = smach.Concurrence(
                outcomes=[TopLevelOutcome.SENSOR_DATA_UPDATE.value],
                default_outcome=TopLevelOutcome.SENSOR_DATA_UPDATE.value,
                input_keys=[
                    SpeedStateDataKeys.ESTOP.value,
                    SpeedStateDataKeys.DISTANCE.value
                    ],
                output_keys=[
                    SpeedStateDataKeys.ESTOP.value,
                    SpeedStateDataKeys.DISTANCE.value
                    ],
                child_termination_cb=self.child_term_cb,
            )

            with monitor_concurrence:
                # Add Monitor States to the fsm to subcribe to ROS topics
                # Topic 1: /proximity_distance
                # Message Type 1: Float32
                smach.Concurrence.add(
                    TopLevelState.MONITOR_PROXIMITY.value,
                    smach_ros.MonitorState(
                        self,
                        topic='/proximity_distance',
                        msg_type=Float32,
                        cond_cb=self.proximity_cb,
                        output_keys=[SpeedStateDataKeys.DISTANCE.value],
                        ))
                # Topic 2: /emergency_stop_status
                # Message Type 2: Bool
                smach.Concurrence.add(
                    TopLevelState.MONITOR_ESTOP.value,
                    smach_ros.MonitorState(
                        self,
                        topic='/emergency_stop_status',
                        msg_type=Bool,
                        cond_cb=self.estop_cb,
                        output_keys=[SpeedStateDataKeys.ESTOP.value]
                        ))

            # Add the concurrent monitoring state to the state machine
            smach.StateMachine.add(
                TopLevelState.CONCURRENT_MONITORING.value,
                monitor_concurrence,
                transitions={
                    TopLevelOutcome.SENSOR_DATA_UPDATE.value:
                    TopLevelState.SPEED_DECISION.value
                    })

            # Add the speed decision state which takes action then switches to
            # monitor proximity state
            smach.StateMachine.add(
                TopLevelState.SPEED_DECISION.value,
                SpeedDecisionState(),
                transitions={
                    SpeedStateOutcomes.STOPPED.value:
                    TopLevelState.STOPPED_STATE.value,
                    SpeedStateOutcomes.SLOW_SPEED.value:
                    TopLevelState.SLOW_SPEED_STATE.value,
                    SpeedStateOutcomes.FULL_SPEED.value:
                    TopLevelState.FULL_SPEED_STATE.value,
                    SpeedStateOutcomes.ESTOP.value:
                    TopLevelState.EMERGENCY_STOP_STATE.value
                }
            )

            smach.StateMachine.add(
                TopLevelState.STOPPED_STATE.value,
                StoppedState(self),
                transitions={
                    SpeedStateOutcomes.DONE.value:
                    TopLevelState.CONCURRENT_MONITORING.value
                    }
            )

            smach.StateMachine.add(
                TopLevelState.SLOW_SPEED_STATE.value,
                SlowSpeedState(self),
                transitions={
                    SpeedStateOutcomes.DONE.value:
                    TopLevelState.CONCURRENT_MONITORING.value
                    }
            )

            smach.StateMachine.add(
                TopLevelState.FULL_SPEED_STATE.value,
                FullSpeedState(self),
                transitions={
                    SpeedStateOutcomes.DONE.value:
                    TopLevelState.CONCURRENT_MONITORING.value
                    }
            )

            smach.StateMachine.add(
                TopLevelState.EMERGENCY_STOP_STATE.value,
                EmergencyStopState(self),
                transitions={
                    SpeedStateOutcomes.DONE.value:
                    TopLevelState.CONCURRENT_MONITORING.value
                    }
            )

        self.fsm.set_initial_state(
            initial_states=[TopLevelState.CONCURRENT_MONITORING.value]
            )

        # Initialize the Estop and proximity distance parameters so that the
        # cobot starts stopped
        setattr(self.fsm.userdata, SpeedStateDataKeys.ESTOP.value, False)
        setattr(self.fsm.userdata, SpeedStateDataKeys.DISTANCE.value, 0)

        # Start the state machine in a separate thread
        self.fsm_thread = threading.Thread(target=self.run_state_machine)
        self.fsm_thread.daemon = True
        self.fsm_thread.start()

        self.get_logger().info(
            f"Speed control node initialized. "
            f"Publishing speed state to /robot_speed_state"
        )

    def run_state_machine(self):
        """Run the state machine in a separate thread"""
        try:
            self.fsm.execute()
        except Exception as e:
            self.get_logger().error(f"State machine error: {e}")

    def proximity_cb(self, userdata, msg):
        '''
        update the distance value then return with invalid
        '''
        self.get_logger().info(f"Proximity distance: {msg.data}")
        userdata.distance = msg.data
        return False

    def estop_cb(self, userdata, msg):
        '''
        update the estop value then return with invalid
        '''
        self.get_logger().info(f"Estop status: {msg.data}")
        userdata.estop = msg.data
        return False

    def child_term_cb(self, outcome_map):
        '''
        Terminate cuncurrent monitoring when ANY sensor updates
        '''
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
