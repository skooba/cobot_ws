#!/usr/bin/env python3
# Standard Python imports
import unittest
from unittest.mock import patch, Mock, call

# Internal imports
from keyboard_controller_pkg.keyboard_input_monitor import (
    KeyboardInputMonitorNode,
)

# ROS2 imports
import rclpy
from rclpy.parameter import Parameter
from example_interfaces.msg import Char


class TestKeyboardInputMonitor(unittest.TestCase):
    def setUp(self):
        """
        Set up ROS2 node for testing
        """
        rclpy.init()

    def tearDown(self):
        """
        Clean up after each test
        """
        rclpy.shutdown()

    def test_parameter_declaration(self):
        """
        Test that the node declares the correct parameters
        """
        # Mock all the problematic methods during node creation
        with patch.object(
            KeyboardInputMonitorNode, "saveTerminalSettings", return_value=None
        ), patch("threading.Thread"), patch.object(
            KeyboardInputMonitorNode, "create_publisher", return_value=Mock()
        ):

            node = KeyboardInputMonitorNode()

            # Test parameter exists with correct default value
            self.assertTrue(node.has_parameter("key_timeout"))
            self.assertEqual(node.get_parameter("key_timeout").value, 0.1)

            node.destroy_node()

    def test_parameter_callback_validation(self):
        """
        Test parameter callback validation logic (unit test)
        """
        # Mock all the problematic methods during node creation
        with patch.object(
            KeyboardInputMonitorNode, "saveTerminalSettings", return_value=None
        ), patch("threading.Thread"), patch.object(
            KeyboardInputMonitorNode, "create_publisher", return_value=Mock()
        ):

            node = KeyboardInputMonitorNode()

            # Test valid parameter
            param_valid = Parameter("key_timeout", Parameter.Type.DOUBLE, 0.5)
            result = node.keyboard_parameters_callback([param_valid])
            self.assertTrue(result.successful)
            self.assertEqual(node.key_timeout, 0.5)

            # Test invalid parameter (negative value)
            param_invalid = Parameter(
                "key_timeout", Parameter.Type.DOUBLE, -1.0
            )
            result = node.keyboard_parameters_callback([param_invalid])
            self.assertFalse(result.successful)
            # Should not update the value for invalid input
            self.assertEqual(node.key_timeout, 0.5)

            node.destroy_node()

    def test_node_name_and_publisher(self):
        """
        Test that the node has correct name and publisher
        """
        with patch.object(
            KeyboardInputMonitorNode, "saveTerminalSettings", return_value=None
        ), patch("threading.Thread"):

            mock_publisher = Mock()
            with patch.object(
                KeyboardInputMonitorNode,
                "create_publisher",
                return_value=mock_publisher,
            ) as mock_create_publisher:
                node = KeyboardInputMonitorNode()

                # Test node name
                self.assertEqual(node.get_name(), "keyboard_input_monitor")

                # Test that our specific publisher was created
                # (check for the Char publisher call)
                expected_call = call(Char, "keyboard_input_monitor", 10)
                self.assertIn(
                    expected_call, mock_create_publisher.call_args_list
                )

                # Verify publisher is assigned
                self.assertEqual(node.publisher_, mock_publisher)

                node.destroy_node()


if __name__ == "__main__":
    unittest.main()
