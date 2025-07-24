#!/usr/bin/env python3
# Standard Python imports
import time
import unittest

# 3rd party imports
from cobot_control_pkg.proximity_sensor import ProximitySensorNode
from example_interfaces.msg import Float32
import rclpy
from rclpy.parameter import Parameter


class TestProximitySensor(unittest.TestCase):
    def setUp(self):
        """Set up ROS2 node for testing."""
        rclpy.init()
        self.node: ProximitySensorNode = ProximitySensorNode()
        self.received_messages = []

    def tearDown(self):
        """Clean up after each test."""
        self.received_messages.clear()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        """Test that the node is initialized correctly."""
        self.assertEqual(self.node.get_name(), 'proximity_sensor')
        self.assertEqual(
            self.node.get_parameter('min_distance_mm').value, 100.0
        )
        self.assertEqual(
            self.node.get_parameter('max_distance_mm').value, 1000.0
        )
        self.assertEqual(
            self.node.get_parameter('publish_rate_hz').value, 0.25
        )
        self.assertIsNotNone(
            self.node.publisher_, 'Publisher should be created'
        )
        self.assertIsNotNone(self.node.timer, 'Timer should be created')

    def test_setting_custom_parameters(self):
        """Test setting custom parameters to the node during runtime."""
        result = self.node.set_parameters(
            [
                Parameter('min_distance_mm', Parameter.Type.DOUBLE, 50.0),
                Parameter('max_distance_mm', Parameter.Type.DOUBLE, 1200.0),
                Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 1.0),
            ]
        )

        # Verify parameters were set successfully
        self.assertTrue(result[0].successful)
        self.assertTrue(result[1].successful)
        self.assertTrue(result[2].successful)

        # Verify the parameters were updated
        self.assertEqual(
            self.node.get_parameter('min_distance_mm').value, 50.0
        )
        self.assertEqual(
            self.node.get_parameter('max_distance_mm').value, 1200.0
        )
        self.assertEqual(self.node.get_parameter('publish_rate_hz').value, 1.0)

        # Verify the instance variables were updated
        self.assertEqual(self.node.min_distance_mm, 50.0)
        self.assertEqual(self.node.max_distance_mm, 1200.0)
        self.assertEqual(self.node.publish_rate_hz, 1.0)

    def test_publish_proximity_data(self):
        """Verify that proximity sensor publishes valid distance readings."""
        # Increase publish rate to get many messages fast
        nominal_frequency = 10.0
        self.node.set_parameters(
            [
                Parameter(
                    'publish_rate_hz', Parameter.Type.DOUBLE, nominal_frequency
                )
            ]
        )

        # Give the parameter callback time to recreate the timer
        rclpy.spin_once(self.node, timeout_sec=0.1)

        message_timestamps = []

        def proximity_callback(msg):
            """Capture published messages."""
            message_timestamps.append(time.time())
            self.received_messages.append(msg.data)

        # Create subscriper to the '/proximity_distance' topic
        self.node.create_subscription(
            Float32, '/proximity_distance', proximity_callback, 10
        )

        # Spin node until enough messages are received or timeout occurs
        start_time = time.time()
        min_expected_messages = 30
        while (
            len(self.received_messages) < min_expected_messages
            and (time.time() - start_time) < 5
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Verify we received enough messages
        num_messages = len(self.received_messages)
        self.assertGreaterEqual(
            num_messages,
            min_expected_messages,
            f'Expected a minimum of {min_expected_messages} but received '
            f'{num_messages}',
        )

        # Calculate the frequency that the messages were being received at
        approximate_freq = (num_messages - 1) / (
            message_timestamps[-1] - message_timestamps[0]
        )

        # Verify the publish rate was correct
        self.assertAlmostEqual(nominal_frequency, approximate_freq, places=1)

        # Verify message values are within expected range
        for distance in self.received_messages:
            self.assertGreaterEqual(distance, self.node.min_distance_mm)
            self.assertLessEqual(distance, self.node.max_distance_mm)

    def test_publish_proximity_data_method(self):
        """Test the publish_proximity_data method directly (unit test)."""
        def direct_callback(msg):
            self.received_messages.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float32, '/proximity_distance', direct_callback, 10
        )

        # Call the method directly
        self.node.publish_proximity_data()

        # Spin once to process the message
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify exactly one message was published
        self.assertEqual(len(self.received_messages), 1)

        # Test multiple calls
        for _ in range(5):
            self.node.publish_proximity_data()
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Should have 6 total messages now (1 + 5)
        self.assertEqual(len(self.received_messages), 6)

    def test_parameter_callback_method(self):
        """Test the parameter_callback method directly (unit test)."""
        test_cases = [
            # Valid parameter changes
            {
                'param': 'min_distance_mm',
                'value': 50.0,
                'should_succeed': True,
            },
            {
                'param': 'max_distance_mm',
                'value': 2000.0,
                'should_succeed': True,
            },
            {'param': 'publish_rate_hz', 'value': 1.0, 'should_succeed': True},
            # Invalid parameter changes
            {
                'param': 'min_distance_mm',
                'value': -1.0,
                'should_succeed': False,
            },  # Invalid negative value
            {
                'param': 'max_distance_mm',
                'value': 1.0,
                'should_succeed': False,
            },  # Invalid: less than than min_distance_mm
            {
                'param': 'publish_rate_hz',
                'value': 0.0,
                'should_succeed': False,
            },  # Invalid zero value
        ]

        for case in test_cases:
            with self.subTest(key=case):
                # Store original values
                original_min_distance_mm = self.node.min_distance_mm
                original_max_distance_mm = self.node.max_distance_mm
                original_publish_rate_hz = self.node.publish_rate_hz

                # Call the callback method directly
                param = Parameter(
                    case['param'], Parameter.Type.DOUBLE, case['value']
                )
                result = self.node.proximity_sensor_parameters_callback(
                    [param]
                )

                # Verify the result matches expectation
                expected_result = (
                    'succeed' if case['should_succeed'] else 'fail'
                )
                self.assertEqual(
                    result.successful,
                    case['should_succeed'],
                    f'Parameter {case["param"]} with value {case["value"]} '
                    f'should {expected_result}',
                )

                if case['should_succeed']:
                    # Verify the parameter was updated
                    if case['param'] == 'min_distance_mm':
                        self.assertEqual(
                            self.node.min_distance_mm, case['value']
                        )
                    elif case['param'] == 'max_distance_mm':
                        self.assertEqual(
                            self.node.max_distance_mm, case['value']
                        )
                    elif case['param'] == 'publish_rate_hz':
                        self.assertEqual(
                            self.node.publish_rate_hz, case['value']
                        )
                else:
                    # Verify the parameter was NOT updated for (invalid)
                    if case['param'] == 'min_distance_mm':
                        self.assertEqual(
                            self.node.min_distance_mm, original_min_distance_mm
                        )
                    elif case['param'] == 'max_distance_mm':
                        self.assertEqual(
                            self.node.max_distance_mm, original_max_distance_mm
                        )
                    elif case['param'] == 'publish_rate_hz':
                        self.assertEqual(
                            self.node.publish_rate_hz, original_publish_rate_hz
                        )


if __name__ == '__main__':
    unittest.main()
