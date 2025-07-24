#!/usr/bin/env python3
# Standard Python imports
import random

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from example_interfaces.msg import Float32


class ProximitySensorNode(Node):

    def __init__(self):
        super().__init__("proximity_sensor")
        # Create some ROS2 parameters that can be adjusted at runtime if needed
        self.declare_parameter("min_distance_mm", 100.0)
        self.declare_parameter("max_distance_mm", 1000.0)
        self.declare_parameter("publish_rate_hz", 0.25)

        self.min_distance_mm = self.get_parameter("min_distance_mm").value
        self.max_distance_mm = self.get_parameter("max_distance_mm").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value

        self.add_on_set_parameters_callback(
            self.proximity_sensor_parameters_callback
        )

        # Create the publisher to publish distance to proximity sensor
        # Topic: /proximity_distance
        # Message Type: example_interfaces/Float32
        self.publisher_ = self.create_publisher(
            Float32, "/proximity_distance", 10
        )
        self.get_logger().info(
            "Proximity sensor node initialized. Publishing on"
            + "/proximity_distance."
        )

        # Create a timer to periodically publish sensor readings
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.publish_proximity_data
        )

    def proximity_sensor_parameters_callback(
        self, params: list[Parameter]
    ) -> SetParametersResult:
        """Callback to handle parameter changes during runtime"""
        result = False
        for param in params:
            if param.name == "min_distance_mm":
                if param.value >= 0 and param.value <= self.max_distance_mm:
                    self.min_distance_mm = param.value
                    self.get_logger().info(
                        f"Updated min_distance_mm to {param.value}"
                    )
                    result = True
                else:
                    self.get_logger().error(
                        f"Invalid min_distance_mm value: {param.value}"
                    )
                    result = False
            if param.name == "max_distance_mm":
                if param.value >= self.min_distance_mm:
                    self.max_distance_mm = param.value
                    self.get_logger().info(
                        f"Updated max_distance_mm to {param.value}"
                    )
                    result = True
                else:
                    self.get_logger().error(
                        f"Invalid max_distance_mm value: {param.value}"
                    )
                    result = False
            if param.name == "publish_rate_hz":
                if param.value > 0:
                    self.publish_rate_hz = param.value
                    # Recreate timer with new frequency
                    self.destroy_timer(self.timer)
                    self.timer = self.create_timer(
                        1.0 / self.publish_rate_hz, self.publish_proximity_data
                    )
                    self.get_logger().info(
                        f"Timer recreated with new rate: "
                        f"{self.publish_rate_hz} Hz"
                    )
                    result = True
                else:
                    self.get_logger().error(
                        f"Invalid publish_rate_hz value: {param.value}"
                    )
                    result = False

        return SetParametersResult(successful=result)

    def publish_proximity_data(self) -> None:
        """Simulate a random proximity reading and publish the reading to the
        'proximity_distance' topic"""
        distance_mm = random.uniform(
            self.min_distance_mm, self.max_distance_mm
        )
        msg = Float32()
        msg.data = distance_mm
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProximitySensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
