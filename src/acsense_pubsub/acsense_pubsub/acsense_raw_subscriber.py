import rclpy
from rclpy.node import Node

from acsense_ros_interfaces.msg import (
    AcSenseRawData,
    AcSenseRawDataSingleChannel,
)


class MinimalRawSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_ac_subscriber")
        self.get_logger().info(f"Setting up AcSense Raw Subscriber")
        self.subscription = self.create_subscription(
            AcSenseRawData, "raw_data", self.listener_callback, 10  # CHANGE
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            (
                f"Received packet no. {msg.packet_num} with {len(msg.array_data)} ch "
                f"x {len(msg.array_data[0].channel_data)} samples per ch"
            )
        )  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalRawSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
