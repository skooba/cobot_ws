#!/usr/bin/env python3
# Standard Python imports
import threading
import sys
from select import select

# Platform-specific imports for terminal settings
if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty

# ROS2 imports
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Char


class KeyboardInputMonitorNode(Node):
    def __init__(self):
        super().__init__("keyboard_input_monitor")
        # Create some ROS2 parameters that can be adjusted at runtime if needed
        self.declare_parameter("key_timeout", 0.1)

        self.key_timeout = self.get_parameter("key_timeout").value

        # Load original keyboard terminal settings
        self.settings = self.saveTerminalSettings()

        # Start a thread to monitor the keyboard input
        self.input_thread = threading.Thread(
            target=self.monitor_keyboard_input_loop, daemon=True
        )
        self.input_thread.start()

        # Create the publisher for which the keyboard presses will be published
        # Topic: /keyboard_input_monitor
        # Message Type: example_interfaces/Char
        self.publisher_ = self.create_publisher(Char, "keyboard_input_monitor", 10)
        self.get_logger().info(
            "Keyboard input monitor node initialized. Publishing on /keyboard_input_monitor."
        )

    def getKey(self) -> str:
        """
        Reads a single character from stdin without blocking indefinitely.
        Adapted from teleop_twist_keyboard.
        """
        if sys.platform == "win32":
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], self.key_timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ""
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def monitor_keyboard_input_loop(self) -> None:
        """
        Main loop for the keyboard monitoring thread.
        Continuously reads keys and updates/publishes chars.
        """
        while rclpy.ok():
            key = self.getKey()
            if key == "\x03":  # Ctrl+C character
                self.get_logger().info(
                    "Ctrl+C detected from keyboard thread. Shutting down get key loop"
                )
                self.restoreTerminalSettings()
                break
            elif key:  # Only publish if we got a key (not empty string)
                # Create a Char message and set its data field
                char_msg = Char()
                char_msg.data = ord(key)  # Convert string to ASCII value
                self.publisher_.publish(char_msg)

    def saveTerminalSettings(self):
        """Saves current terminal settings."""
        if sys.platform == "win32":
            return None
        return termios.tcgetattr(sys.stdin)

    def restoreTerminalSettings(self) -> None:
        """Restores original terminal settings."""
        if sys.platform == "win32":
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
