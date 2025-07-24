#!/usr/bin/env python3
# Standard Python imports
import unittest
from unittest.mock import Mock

# 3rd party imports
from cobot_control_pkg.state_machine.speed_state_machine import (
    EmergencyStopState,
    FullSpeedState,
    SlowSpeedState,
    SpeedControlVariables,
    SpeedDecisionState,
    SpeedStateDataKeys,
    SpeedStateOutcomes,
    StoppedState,
)
import smach


class TestSpeedStateMachine(unittest.TestCase):
    def setUp(self):
        """Set up ROS2 node for testing."""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value.info = Mock()
        self.mock_node.speed_state_publisher_ = Mock()

    def test_speed_decision_emergency_stop_priority(self):
        """Test that emergency stop takes priority over all other conditions."""
        state = SpeedDecisionState()

        test_cases = [
            {
                'estop': True,
                'distance': 100.0,
                'expected': SpeedStateOutcomes.ESTOP.value,
            },
            {
                'estop': True,
                'distance': 500.0,
                'expected': SpeedStateOutcomes.ESTOP.value,
            },
            {
                'estop': True,
                'distance': 1000.0,
                'expected': SpeedStateOutcomes.ESTOP.value,
            },
        ]

        for case in test_cases:
            with self.subTest(estop=case['estop'], distance=case['distance']):
                userdata = smach.UserData()
                userdata.estop = case['estop']
                userdata.distance = case['distance']

                result = state.execute(userdata)
                self.assertEqual(result, case['expected'])

    def test_speed_decision_distance_based_logic(self):
        """Test speed decision logic based on distance thresholds."""
        state = SpeedDecisionState()

        test_cases = [
            {
                'distance': 300.0,
                'expected': SpeedStateOutcomes.STOPPED.value,
                'description': 'close range',
            },
            {
                'distance': 600.0,
                'expected': SpeedStateOutcomes.SLOW_SPEED.value,
                'description': 'medium range',
            },
            {
                'distance': 900.0,
                'expected': SpeedStateOutcomes.FULL_SPEED.value,
                'description': 'safe range',
            },
        ]

        for case in test_cases:
            with self.subTest(distance=case['distance']):
                userdata = smach.UserData()
                userdata.estop = False
                userdata.distance = case['distance']

                result = state.execute(userdata)
                self.assertEqual(
                    result,
                    case['expected'],
                    f'Distance {case["distance"]} should result in '
                    f' {case["description"]}',
                )

    def test_speed_decision_boundary_conditions(self):
        """Test boundary conditions for distance thresholds."""
        state = SpeedDecisionState()
        userdata = smach.UserData()
        userdata.estop = False

        # Test exactly at boundaries
        userdata.distance = SpeedControlVariables.MIN_SLOW_SPEED_DISTANCE.value
        result = state.execute(userdata)
        self.assertEqual(result, SpeedStateOutcomes.SLOW_SPEED.value)

        userdata.distance = SpeedControlVariables.MAX_SLOW_SPEED_DISTANCE.value
        result = state.execute(userdata)
        self.assertEqual(result, SpeedStateOutcomes.SLOW_SPEED.value)

        # Test just below boundaries
        userdata.distance = (
            SpeedControlVariables.MIN_SLOW_SPEED_DISTANCE.value - 0.1
        )
        result = state.execute(userdata)
        self.assertEqual(result, SpeedStateOutcomes.STOPPED.value)

        userdata.distance = (
            SpeedControlVariables.MAX_SLOW_SPEED_DISTANCE.value + 0.1
        )
        result = state.execute(userdata)
        self.assertEqual(result, SpeedStateOutcomes.FULL_SPEED.value)

    def test_individual_state_execution_with_node(self):
        """Test individual state execution when node is provided."""
        test_cases = [
            (StoppedState, SpeedStateOutcomes.STOPPED.value),
            (SlowSpeedState, SpeedStateOutcomes.SLOW_SPEED.value),
            (FullSpeedState, SpeedStateOutcomes.FULL_SPEED.value),
            (EmergencyStopState, SpeedStateOutcomes.ESTOP.value),
        ]

        for state_class, expected_outcome in test_cases:
            with self.subTest(state=state_class.__name__):
                state = state_class(self.mock_node)
                userdata = smach.UserData()

                result = state.execute(userdata)
                self.assertEqual(result, SpeedStateOutcomes.DONE.value)

                # Verify publisher was called
                publisher = self.mock_node.speed_state_publisher_
                publisher.publish.assert_called_once()
                publisher.publish.reset_mock()

    def test_individual_state_execution_without_node(self):
        """Test individual state execution when no node is provided."""
        test_cases = [
            StoppedState,
            SlowSpeedState,
            FullSpeedState,
            EmergencyStopState,
        ]

        for state_class in test_cases:
            with self.subTest(state=state_class.__name__):
                state = state_class(None)
                userdata = smach.UserData()

                result = state.execute(userdata)
                self.assertEqual(result, SpeedStateOutcomes.DONE.value)

    def test_speed_control_variables_constants(self):
        """Test that speed control constants are correct."""
        self.assertEqual(
            SpeedControlVariables.MAX_SLOW_SPEED_DISTANCE.value, 800
        )
        self.assertEqual(
            SpeedControlVariables.MIN_SLOW_SPEED_DISTANCE.value, 400
        )

    def test_state_data_keys_enum(self):
        """Test that state data keys are correctly defined."""
        self.assertEqual(SpeedStateDataKeys.ESTOP.value, 'estop')
        self.assertEqual(SpeedStateDataKeys.DISTANCE.value, 'distance')


if __name__ == '__main__':
    unittest.main()
