#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Char


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("emergency_stop")

        # Create the subscriber for which the keyboard keys will be recognized
        # Topic: /keyboard_input_monitor
        # Message Type: example_interfaces/Char
        self.subscriber_ = self.create_subscription(
            Char, "keyboard_input_monitor", self.callback_estop, 10
        )

        # Create the publisher for the emergency stop to be published on
        # Topic: /emergency_stop_status
        # Message Type: example_interfaces/Bool
        self.e_stop_active = False
        self.publisher_ = self.create_publisher(
            Bool, "emergency_stop_status", 10
        )
        self.publish_estop_data()

        self.get_logger().info(
            "Open the keyboard_input_monitor_node from the"
            "keyboard_controller_pkg and press 'e' to "
            "activate E-Stop, 'r' to reset E-Stop.\n Press "
            "Ctrl+C toexit."
        )

    def callback_estop(self, msg: Char) -> None:
        if msg.data == ord("r"):
            self.toggle_estop(False)
        elif msg.data == ord("e"):
            self.toggle_estop(True)
        else:
            pass

    def toggle_estop(self, state: bool) -> None:
        """
        Toggle the estop and publish the value as soon as this function is
        called
        """
        self.e_stop_active = state
        self.publish_estop_data()

    def publish_estop_data(self) -> None:
        """publish the estop value to the 'emergency_stop_status' topic"""
        msg = Bool()
        msg.data = self.e_stop_active
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
