#!/usr/bin/env python3
# Standard Python imports
import time
import unittest

# Third Party imports
from cobot_control_pkg.speed_control import SpeedControlNode
from cobot_control_pkg.state_machine.speed_state_machine import (
    SpeedStateDataKeys,
    SpeedStateOutcomes,
)
from example_interfaces.msg import Bool, Float32, String
import rclpy
import smach


class TestSpeedControl(unittest.TestCase):
    def setUp(self):
        """Set up ROS2 node for testing."""
        rclpy.init()
        self.node: SpeedControlNode = SpeedControlNode()
        self.received_speed_states = []

    def tearDown(self):
        """Clean up after each test."""
        if self.node.fsm_thread.is_alive():
            # Try to stop the state machine gracefully
            try:
                self.node.fsm.request_preempt()
            except Exception:
                pass
            finally:
                self.node.fsm_thread.join(timeout=1.0)
        self.received_speed_states.clear()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        """Test that speed control node initializes correctly."""
        self.assertEqual(self.node.get_name(), 'speed_controller')
        self.assertIsNotNone(self.node.speed_state_publisher_)
        self.assertIsNotNone(self.node.fsm)
        self.assertIsNotNone(self.node.fsm_thread)
        self.assertTrue(self.node.fsm_thread.is_alive())

    def test_proximity_callback_functionality(self):
        """Test proximity sensor callback updates userdata correctly."""
        # Create test userdata
        userdata = smach.UserData()
        userdata.distance = 0.0

        # Test with different distances
        test_distances = [100.0, 500.0, 900.0]

        for distance in test_distances:
            with self.subTest(distance=distance):
                # Create test message
                msg = Float32()
                msg.data = distance

                # Call callback
                result = self.node.proximity_cb(userdata, msg)

                # Verify userdata was updated
                self.assertEqual(userdata.distance, distance)
                # Callback should return False
                # (invalid condition for monitor state)
                self.assertFalse(result)

    def test_estop_callback_functionality(self):
        """Test emergency stop callback updates userdata correctly."""
        # Create test userdata
        userdata = smach.UserData()
        userdata.estop = False

        # Test estop activation
        msg = Bool()
        msg.data = True
        result = self.node.estop_cb(userdata, msg)

        self.assertTrue(userdata.estop)
        self.assertFalse(result)  # Should return False for monitor state

        # Test estop deactivation
        msg.data = False
        result = self.node.estop_cb(userdata, msg)

        self.assertFalse(userdata.estop)
        self.assertFalse(result)

    def test_child_termination_callback(self):
        """Test child termination callback always returns True."""
        # Create mock outcome map
        outcome_map = {'monitor_prox': 'invalid', 'monitor_estop': 'invalid'}

        result = self.node.child_term_cb(outcome_map)

        # Should always return True to terminate concurrent monitoring
        self.assertTrue(result)

    def test_speed_state_publishing(self):
        """Test that speed states are published correctly based on proximity input."""

        def speed_state_callback(msg):
            self.received_speed_states.append(msg.data)

        # Create subscriber for speed states
        self.node.create_subscription(
            String, '/robot_speed_state', speed_state_callback, 10
        )

        # Create publisher to simulate proximity sensor
        proximity_publisher = self.node.create_publisher(
            Float32, '/proximity_distance', 10
        )

        # Create publisher to simulate estop
        self.node.create_publisher(Bool, '/emergency_stop_status', 10)

        # Test different proximity scenarios
        test_scenarios = [
            {
                'distance': 300.0,
                'expected_state': SpeedStateOutcomes.STOPPED.value,
                'description': 'close range',
            },
            {
                'distance': 600.0,
                'expected_state': SpeedStateOutcomes.SLOW_SPEED.value,
                'description': 'medium range',
            },
            {
                'distance': 900.0,
                'estop': False,
                'expected_state': SpeedStateOutcomes.FULL_SPEED.value,
                'description': 'safe range',
            },
        ]

        for scenario in test_scenarios:
            with self.subTest(
                distance=scenario['distance'],
            ):
                # Clear previous messages
                self.received_speed_states.clear()

                # Publish proximity distance
                proximity_msg = Float32()
                proximity_msg.data = scenario['distance']
                proximity_publisher.publish(proximity_msg)

                start_time = time.time()
                while (
                    len(self.received_speed_states) < 1
                    and (time.time() - start_time) < 5.0
                ):
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                # Verify we received a speed state
                self.assertGreater(
                    len(self.received_speed_states),
                    0,
                    f"No speed state received for {scenario['description']}",
                )

                # Verify the correct state was published
                self.assertEqual(
                    self.received_speed_states[-1],
                    scenario['expected_state'],
                    f'Expected {scenario["expected_state"]} for '
                    f' {scenario["description"]}',
                )

                rclpy.spin_once(self.node, timeout_sec=0.2)

    def test_fsm_initial_state(self):
        """Test that FSM initializes with correct initial state."""
        # Check that userdata has initial values
        self.assertFalse(
            self.node.fsm.userdata[SpeedStateDataKeys.ESTOP.value]
        )
        self.assertEqual(
            self.node.fsm.userdata[SpeedStateDataKeys.DISTANCE.value], 0
        )

    def test_state_machine_thread_safety(self):
        """Test that state machine runs safely in separate thread."""
        # Verify thread properties
        self.assertTrue(self.node.fsm_thread.is_alive())
        self.assertTrue(self.node.fsm_thread.daemon)

        # Let it run for a short time
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Thread should still be alive
        self.assertTrue(self.node.fsm_thread.is_alive())


if __name__ == '__main__':
    unittest.main()
