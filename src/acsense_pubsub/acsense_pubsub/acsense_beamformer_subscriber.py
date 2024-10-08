import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore

from acsense_ros_interfaces.msg import (  # type: ignore
    AcSenseBeamformerData,
)


class MinimalBeamformerRawSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_beamformer_subscriber")
        self.get_logger().info("Setting up AcSense Beamformer Data Subscriber")
        self.subscription = self.create_subscription(
            AcSenseBeamformerData,
            "beamformer_data",
            self.listener_callback,
            10,  # CHANGE
        )

    def listener_callback(self, msg):
        data = np.array(msg.beamformer_response).reshape(msg.shape)
        self.get_logger().info(
            (
                f"Received packet no. {msg.packet_num} with {msg.num_frequencies} freq "
                f"x {msg.num_bearings} bearings "
                f"x {msg.num_elevations} elevations "
                f"{data.shape}"
            )
        )  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalBeamformerRawSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
