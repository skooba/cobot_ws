#!/usr/bin/env python3
# Standard Python imports
import unittest
import time

# Internal imports
from cobot_control_pkg.emergency_stop import EmergencyStopNode

# ROS2 imports
import rclpy
from example_interfaces.msg import Char, Bool

class TestEmergencyStop(unittest.TestCase):
    def setUp(self):
        """
        Set up ROS2 node for testing
        """
        rclpy.init()
        # Create subscriber to capture emergency stop messages
        self.node: EmergencyStopNode = EmergencyStopNode()
        self.received_estop_messages = []

    def tearDown(self):
        """
        Clean up after each test
        """
        self.received_estop_messages.clear()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        """
        Test that the emergency stop node initializes correctly
        """
        self.assertEqual(self.node.get_name(), "emergency_stop")
        self.assertFalse(self.node.e_stop_active, "Initial state of the estop should be active")
        self.assertIsNotNone(self.node.publisher_, "Publisher should be created")

    def test_emergency_stop_activation(self):
        '''
        Test emergency stop activates with the 'e' key
        '''
        def estop_callback(msg):
            self.received_estop_messages.append(msg.data)

        # Create subscriber to capture emergency stop messages
        self.node.create_subscription(
            Bool, '/emergency_stop_status', estop_callback, 10
        )

        # Simulate 'e' key press
        char_msg = Char()
        char_msg.data = ord('e')
        self.node.callback_estop(char_msg)

        # Spin to process the message
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify emergency stop is activated
        self.assertTrue(self.node.e_stop_active, "Emergency stop should be active")
        self.assertGreater(len(self.received_estop_messages), 0, "No estop message received")
        self.assertTrue(self.received_estop_messages[-1], "Published estop message should be True")

    def test_emergency_stop_reset(self):
        '''
        Test emergency stop resets with the 'r' key
        '''
        def estop_callback(msg):
            self.received_estop_messages.append(msg.data)

        # Create subscriber to capture emergency stop messages
        self.node.create_subscription(
            Bool, '/emergency_stop_status', estop_callback, 10
        )

        # Simulate 'e' key press
        char_msg = Char()
        char_msg.data = ord('r')
        self.node.callback_estop(char_msg)

        # Spin to process the message
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify emergency stop is activated
        self.assertFalse(self.node.e_stop_active, "Emergency stop should be reset")
        self.assertGreater(len(self.received_estop_messages), 0, "No estop message received")
        self.assertFalse(self.received_estop_messages[-1], "Published estop message should be False")


    def test_invalid_character_handeling(self):
        '''
        Tests that invalid keys are ignored
        '''
        initial_state = self.node.e_stop_active
        
        # Test various invalid keys
        invalid_keys = ['x', 'q', '1', ' ', '\n']
        
        for key in invalid_keys:
            with self.subTest(key=key):
                char_msg = Char()
                char_msg.data = ord(key)
                self.node.callback_estop(char_msg)
                
                # State should remain unchanged
                self.assertEqual(self.node.e_stop_active, initial_state, 
                               f"State should not change for invalid key '{key}'")

    def test_callback_estop_method(self):
        '''
        Tests the calback_estop method directly (Unit Test)
        '''
        test_cases = [
            {'key': 'e', 'expected_state': True, 'description': 'activate'},
            {'key': 'r', 'expected_state': False, 'description': 'reset'},
            {'key': 'x', 'expected_state': False, 'description': 'ignore invalid'}
        ]

        for case in test_cases:
            with self.subTest(key=case['key']):
                # Reset to known state
                self.node.e_stop_active = False
                
                # Create and send message
                char_msg = Char()
                char_msg.data = ord(case['key'])
                self.node.callback_estop(char_msg)
                
                # Verify result
                self.assertEqual(self.node.e_stop_active, case['expected_state'],
                               f"Should {case['description']} estop with '{case['key']}'")

    def test_toggle_estop_method(self):
        '''
        Tests the toggle_estop method directly (Unit Test)
        '''
        # Test activation
        self.node.toggle_estop(True)
        self.assertTrue(self.node.e_stop_active, "toggle_estop(True) should activate estop")

        # Test deactivation
        self.node.toggle_estop(False)
        self.assertFalse(self.node.e_stop_active, "toggle_estop(False) should deactivate estop")

    def test_publish_estop_data_method(self):
        '''
        Tests the publish_estop_data method directly (Unit Test)
        '''
        def estop_callback(msg):
            self.received_estop_messages.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Bool, '/emergency_stop_status', estop_callback, 10
        )

        # Test publishing when estop is False
        self.node.e_stop_active = False
        self.node.publish_estop_data()
        rclpy.spin_once(self.node, timeout_sec=0.5)
        
        self.assertFalse(self.received_estop_messages[-1], "Should publish False when estop inactive")

        # Test publishing when estop is True
        self.node.e_stop_active = True
        self.node.publish_estop_data()
        rclpy.spin_once(self.node, timeout_sec=0.5)
        
        self.assertTrue(self.received_estop_messages[-1], "Should publish True when estop active")

    def test_emergency_stop_workflow_sequence(self):
        '''
        Test complete emergency stop activation and reset workflow"
        '''
        def estop_callback(msg):
            self.received_estop_messages.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Bool, '/emergency_stop_status', estop_callback, 10
        )

        # Test sequence: activate → reset → activate → reset
        test_cases = [
            {'key': 'e', 'expected_state': True},
            {'key': 'r', 'expected_state': False},
            {'key': 'e', 'expected_state': True},
            {'key': 'r', 'expected_state': False}
        ]

        for case in test_cases:
            with self.subTest(
                key=case['key'], 
                expected=case['expected_state']
            ):
                # Send key
                char_msg = Char()
                char_msg.data = ord(case['key'])
                self.node.callback_estop(char_msg)
                
                # Process message
                rclpy.spin_once(self.node, timeout_sec=0.5)
                
                # Verify state
                self.assertEqual(self.node.e_stop_active, case['expected_state'],
                               f"State should be {case['expected_state']} after '{case['key']}'")

    def test_multiple_activations_and_resets(self):
        """Test that multiple consecutive activations/resets work correctly"""
        # Multiple activations should keep it active
        for i in range(3):
            char_msg = Char()
            char_msg.data = ord('e')
            self.node.callback_estop(char_msg)
            self.assertTrue(self.node.e_stop_active, f"Should remain active after activation {i+1}")

        # Multiple resets should keep it inactive
        for i in range(3):
            char_msg = Char()
            char_msg.data = ord('r')
            self.node.callback_estop(char_msg)
            self.assertFalse(self.node.e_stop_active, f"Should remain inactive after reset {i+1}")