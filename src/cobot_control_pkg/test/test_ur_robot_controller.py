#!/usr/bin/env python3
# Standard Python imports
import unittest
import time

# Internal imports
from cobot_control_pkg.ur_robot_controller import (
    URRobotController,
    VelocityValues,
)
from cobot_control_pkg.state_machine.speed_state_machine import (
    SpeedStateOutcomes,
)

# ROS2 imports
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from example_interfaces.msg import String


class TestURRobotController(unittest.TestCase):
    def setUp(self):
        """
        Set up ROS2 node for testing
        """
        rclpy.init()
        self.node: URRobotController = URRobotController()
        self.received_velocity_commands = []

    def tearDown(self):
        """
        Clean up after each test
        """
        self.received_velocity_commands.clear()
        if self.node.ramp_thread and self.node.ramp_thread.is_alive():
            self.node.stop_ramp_event.set()
            self.node.ramp_thread.join(timeout=1.0)
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        """
        Test that the UR robot controller node initializes correctly
        """
        self.assertEqual(self.node.get_name(), "ur_robot_controller")
        self.assertEqual(self.node.get_parameter("ramp_rate").value, 0.5)
        self.assertEqual(self.node.get_parameter("frequency").value, 10.0)
        self.assertIsNotNone(self.node.velocity_publisher_)
        self.assertIsNotNone(self.node.speed_state_subscriber)
        self.assertEqual(len(self.node.joint_names), 6)
        self.assertEqual(
            self.node.current_speed_state, SpeedStateOutcomes.STOPPED.value
        )
        self.assertFalse(self.node.estop_active)
        self.assertEqual(len(self.node.current_velocities), 6)
        self.assertEqual(self.node.current_velocities, [0.0] * 6)

    def test_joint_names_configuration(self):
        """
        Test that joint names are correctly configured for UR5
        """
        expected_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.assertEqual(self.node.joint_names, expected_joints)

    def test_velocity_values_enum(self):
        """
        Test that velocity value constants are correct
        """
        self.assertEqual(VelocityValues.SLOW_SPEED.value, 0.3)
        self.assertEqual(VelocityValues.FULL_SPEED.value, 1.0)

    def test_setting_custom_parameters(self):
        """
        Test setting custom parameters to the node during runtime
        """
        result = self.node.set_parameters(
            [
                Parameter("ramp_rate", Parameter.Type.DOUBLE, 1.0),
                Parameter("frequency", Parameter.Type.DOUBLE, 20.0),
                Parameter("slow_speed", Parameter.Type.DOUBLE, 0.5),
                Parameter("full_speed", Parameter.Type.DOUBLE, 1.5),
            ]
        )

        # Verify parameters were set successfully
        self.assertTrue(result[0].successful)
        self.assertTrue(result[1].successful)
        self.assertTrue(result[2].successful)
        self.assertTrue(result[3].successful)

        # Verify the parameters were updated
        self.assertEqual(self.node.get_parameter("ramp_rate").value, 1.0)
        self.assertEqual(self.node.get_parameter("frequency").value, 20.0)
        self.assertEqual(self.node.get_parameter("slow_speed").value, 0.5)
        self.assertEqual(self.node.get_parameter("full_speed").value, 1.5)

        # Verify the instance variables were updated
        self.assertEqual(self.node.ramp_rate, 1.0)
        self.assertEqual(self.node.frequency, 20.0)
        self.assertEqual(self.node.slow_speed, 0.5)
        self.assertEqual(self.node.full_speed, 1.5)

    def test_parameter_callback_method(self):
        """
        Test the ur_robot_parameters_callback method directly (unit test)
        """
        test_cases = [
            # Valid parameter changes
            {"param": "ramp_rate", "value": 2.0, "should_succeed": True},
            {"param": "frequency", "value": 15.0, "should_succeed": True},
            {"param": "slow_speed", "value": 0.2, "should_succeed": True},
            {"param": "full_speed", "value": 1.5, "should_succeed": True},
            # Invalid parameter changes
            {
                "param": "ramp_rate",
                "value": -1.0,
                "should_succeed": False,
            },  # Invalid negative value
            {
                "param": "frequency",
                "value": 0.0,
                "should_succeed": False,
            },  # Invalid zero value
            {
                "param": "slow_speed",
                "value": -0.1,
                "should_succeed": False,
            },  # Invalid negative value
            {
                "param": "slow_speed",
                "value": 2.0,
                "should_succeed": False,
            },  # Invalid: greater than full_speed
        ]

        for case in test_cases:
            with self.subTest(param=case["param"], value=case["value"]):
                # Store original values
                original_ramp_rate = self.node.ramp_rate
                original_frequency = self.node.frequency
                original_slow_speed = self.node.slow_speed
                original_full_speed = self.node.full_speed

                # Call the callback method directly
                param = Parameter(
                    case["param"], Parameter.Type.DOUBLE, case["value"]
                )
                result = self.node.ur_robot_parameters_callback([param])

                # Verify the result matches expectation
                expected_result = (
                    "succeed" if case["should_succeed"] else "fail"
                )
                self.assertEqual(
                    result.successful,
                    case["should_succeed"],
                    f"Parameter {case['param']} with value {case['value']} "
                    f"should {expected_result}",
                )

                if case["should_succeed"]:
                    # Verify the instance variable was updated for
                    # valid parameters
                    if case["param"] == "ramp_rate":
                        self.assertEqual(self.node.ramp_rate, case["value"])
                    elif case["param"] == "frequency":
                        self.assertEqual(self.node.frequency, case["value"])
                    elif case["param"] == "slow_speed":
                        self.assertEqual(self.node.slow_speed, case["value"])
                    elif case["param"] == "full_speed":
                        self.assertEqual(self.node.full_speed, case["value"])
                else:
                    # Verify the instance variable was NOT updated for
                    #  invalid parameters
                    if case["param"] == "ramp_rate":
                        self.assertEqual(
                            self.node.ramp_rate, original_ramp_rate
                        )
                    elif case["param"] == "frequency":
                        self.assertEqual(
                            self.node.frequency, original_frequency
                        )
                    elif case["param"] == "slow_speed":
                        self.assertEqual(
                            self.node.slow_speed, original_slow_speed
                        )
                    elif case["param"] == "full_speed":
                        self.assertEqual(
                            self.node.full_speed, original_full_speed
                        )

    def test_speed_state_callback_stopped(self):
        """
        Test speed state callback for the stopped state
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber to capture velocity commands
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Set current velocities to non-zero
        self.node.current_velocities = [0.1] * 6

        # Create test speed state message
        speed_msg = String()
        speed_msg.data = SpeedStateOutcomes.STOPPED.value
        self.node.speed_state_callback(speed_msg)

        # Spin to process velocity command
        start_time = time.time()
        while (time.time() - start_time) < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Verify stopped velocities were published
        self.assertGreater(
            len(self.received_velocity_commands),
            0,
            "No velocity command received",
        )
        self.assertEqual(
            list(self.received_velocity_commands[-1]),
            [0.0] * 6,
            "Should publish zero velocities for stopped state",
        )
        self.assertFalse(
            self.node.estop_active,
            "Estop should be inactive after stopped state",
        )

    def test_speed_state_callback_slow_speed(self):
        """
        Test speed state callback for slow speed state
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Send slow speed command
        speed_msg = String()
        speed_msg.data = SpeedStateOutcomes.SLOW_SPEED.value
        self.node.speed_state_callback(speed_msg)

        # Spin to process
        start_time = time.time()
        while (time.time() - start_time) < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Verify slow speed velocities were published
        self.assertGreater(
            len(self.received_velocity_commands),
            0,
            "No velocity command received",
        )
        expected_velocity = [VelocityValues.SLOW_SPEED.value] * 6
        self.assertEqual(
            list(self.received_velocity_commands[-1]),
            expected_velocity,
            "Should publish slow speed velocities",
        )
        self.assertFalse(
            self.node.estop_active,
            "Estop should be inactive after slow speed state",
        )

    def test_speed_state_callback_full_speed(self):
        """
        Test speed state callback for full speed state
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Set current velocities close to full speed
        self.node.current_velocities = [0.8] * 6

        # Send full speed command
        speed_msg = String()
        speed_msg.data = SpeedStateOutcomes.FULL_SPEED.value
        self.node.speed_state_callback(speed_msg)

        # Spin to process
        start_time = time.time()
        while (time.time() - start_time) < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Verify full speed velocities were published
        self.assertGreater(
            len(self.received_velocity_commands),
            0,
            "No velocity command received",
        )
        expected_velocity = [VelocityValues.FULL_SPEED.value] * 6
        self.assertEqual(
            list(self.received_velocity_commands[-1]),
            expected_velocity,
            "Should publish full speed velocities",
        )
        self.assertFalse(
            self.node.estop_active,
            "Estop should be inactive after full speed state",
        )

    def test_speed_state_callback_emergency_stop(self):
        """
        Test speed state callback for emergency stop state
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Send emergency stop command
        speed_msg = String()
        speed_msg.data = SpeedStateOutcomes.ESTOP.value
        self.node.speed_state_callback(speed_msg)

        # Only 1 command should be sent on an emergency stop condition
        while len(self.received_velocity_commands) < 1:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Verify emergency stop behavior
        self.assertGreater(
            len(self.received_velocity_commands),
            0,
            "No velocity command received",
        )
        self.assertEqual(
            list(self.received_velocity_commands[-1]),
            [0.0] * 6,
            "Should publish zero velocities for estop",
        )
        self.assertTrue(
            self.node.estop_active, "Estop should be active after estop state"
        )
        self.assertEqual(
            self.node.current_velocities,
            [0.0] * 6,
            "Current velocities should be zero",
        )

    def test_send_immediate_velocity_command_valid(self):
        """
        Test sending immediate velocity command with valid input
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Test valid velocity command
        test_velocities = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        self.node.send_immediate_velocity_command(test_velocities)

        # Spin to process
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify command was published
        self.assertGreater(
            len(self.received_velocity_commands),
            0,
            "No velocity command received",
        )
        self.assertEqual(
            list(self.received_velocity_commands[-1]),
            test_velocities,
            "Published velocities should match input",
        )

    def test_send_immediate_velocity_command_invalid_length(self):
        """
        Test sending immediate velocity command with invalid length
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Test invalid velocity command (wrong length)
        test_velocities = [0.1, 0.2, 0.3]  # Only 3 values instead of 6
        self.node.send_immediate_velocity_command(test_velocities)

        # Spin to process
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify no command was published due to invalid length
        self.assertEqual(
            len(self.received_velocity_commands),
            0,
            "No velocity command should be published for invalid length",
        )

    def test_estop_robot_method(self):
        """
        Test the estop_robot method directly
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Set some non-zero current velocities
        self.node.current_velocities = [0.5] * 6

        # Call estop method
        self.node.estop_robot()

        # Spin to process
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify estop behavior
        self.assertTrue(self.node.estop_active, "Estop should be active")
        self.assertEqual(
            self.node.current_velocities,
            [0.0] * 6,
            "Current velocities should be zero",
        )
        self.assertGreater(
            len(self.received_velocity_commands),
            0,
            "Should publish a zero velocity command",
        )
        self.assertEqual(
            list(self.received_velocity_commands[-1]),
            [0.0] * 6,
            "Should publish zero velocities",
        )

    def test_velocity_ramping_basic(self):
        """
        Test basic velocity ramping functionality
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Test ramping from zero to slow speed
        target_velocities = [VelocityValues.SLOW_SPEED.value] * 6
        self.node.send_ramped_velocity_command(target_velocities)

        # Wait for ramping to complete
        start_time = time.time()
        while (time.time() - start_time) < 1.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Verify ramping occurred (should receive multiple velocity commands)
        self.assertGreater(
            len(self.received_velocity_commands),
            1,
            "Should receive multiple velocity commands during ramping",
        )

        # Final velocity should be close to target
        if len(self.received_velocity_commands) > 0:
            final_velocity = self.received_velocity_commands[-1]
            for i, vel in enumerate(final_velocity):
                self.assertAlmostEqual(
                    vel,
                    target_velocities[i],
                    places=1,
                    msg=f"Final velocity joint {i} should reach target",
                )

    def test_velocity_ramping_thread_safety(self):
        """
        Test that velocity ramping is thread-safe
        """
        # Start multiple ramping commands
        target1 = [0.2] * 6
        target2 = [0.5] * 6

        self.node.send_ramped_velocity_command(target1)
        rclpy.spin_once(self.node, timeout_sec=0.1)  # Small delay
        self.node.send_ramped_velocity_command(
            target2
        )  # Should stop first ramp and start second

        # Let ramping complete
        rclpy.spin_once(self.node, timeout_sec=1.0)

        # Final velocities should be close to target2
        for i, vel in enumerate(self.node.current_velocities):
            self.assertAlmostEqual(
                vel,
                target2[i],
                places=1,
                msg=f"Final velocity joint {i} should reach second target",
            )

    def test_speed_state_transitions_sequence(self):
        """
        Test sequence of speed state transitions
        """

        def velocity_callback(msg):
            self.received_velocity_commands.append(msg.data)

        # Create subscriber
        self.node.create_subscription(
            Float64MultiArray,
            "forward_velocity_controller/commands",
            velocity_callback,
            10,
        )

        # Test sequence of state transitions
        test_states = [
            SpeedStateOutcomes.SLOW_SPEED.value,
            SpeedStateOutcomes.FULL_SPEED.value,
            SpeedStateOutcomes.STOPPED.value,
            SpeedStateOutcomes.ESTOP.value,
            SpeedStateOutcomes.FULL_SPEED.value,  # After estop reset
        ]

        for i, state in enumerate(test_states):
            with self.subTest(state=state, sequence_number=i):
                # Clear previous messages
                self.received_velocity_commands.clear()

                # Send state command
                speed_msg = String()
                speed_msg.data = state
                self.node.speed_state_callback(speed_msg)

                # Wait for command to be processed
                start_time = time.time()
                while (
                    len(self.received_velocity_commands) < 1
                    and (time.time() - start_time) < 3.0
                ):
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                # Verify state was processed
                self.assertGreater(
                    len(self.received_velocity_commands),
                    0,
                    f"No velocity command received for state {state}",
                )

                # Verify correct estop state
                if state == SpeedStateOutcomes.ESTOP.value:
                    self.assertTrue(
                        self.node.estop_active,
                        f"Estop should be active for {state}",
                    )
                elif state in [
                    SpeedStateOutcomes.STOPPED.value,
                    SpeedStateOutcomes.SLOW_SPEED.value,
                    SpeedStateOutcomes.FULL_SPEED.value,
                ]:
                    self.assertFalse(
                        self.node.estop_active,
                        f"Estop should be inactive for {state}",
                    )

                # Small delay between states
                rclpy.spin_once(self.node, timeout_sec=0.2)

    def test_current_speed_state_tracking(self):
        """
        Test that current speed state is correctly tracked
        """
        # Test different states
        test_states = [
            SpeedStateOutcomes.SLOW_SPEED.value,
            SpeedStateOutcomes.FULL_SPEED.value,
            SpeedStateOutcomes.STOPPED.value,
            SpeedStateOutcomes.ESTOP.value,
        ]

        for state in test_states:
            with self.subTest(state=state):
                speed_msg = String()
                speed_msg.data = state
                self.node.speed_state_callback(speed_msg)

                # Verify current state is updated
                self.assertEqual(
                    self.node.current_speed_state,
                    state,
                    f"Current speed state should be {state}",
                )

                # Small delay between state transitions
                rclpy.spin_once(self.node, timeout_sec=0.2)

    def test_thread_management(self):
        """
        Test that ramping threads are properly managed
        """
        # Initially no ramp thread
        self.assertIsNone(self.node.ramp_thread)

        # Start ramping
        self.node.send_ramped_velocity_command([0.5] * 6)

        # Should have a ramp thread now
        self.assertIsNotNone(self.node.ramp_thread)

        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Start another ramping (should stop previous thread)
        old_thread = self.node.ramp_thread
        self.node.send_ramped_velocity_command([1.0] * 6)

        # Should have a new thread
        self.assertIsNotNone(self.node.ramp_thread)
        self.assertNotEqual(
            old_thread, self.node.ramp_thread, "Should create new thread"
        )


if __name__ == "__main__":
    unittest.main()
