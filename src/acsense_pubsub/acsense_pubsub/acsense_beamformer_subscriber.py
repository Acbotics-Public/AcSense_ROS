import argparse
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore

from acsense_ros_interfaces.msg import (  # type: ignore
    AcSenseBeamformerData,
)


class MinimalBeamformerRawSubscriber(Node):
    def __init__(self, args):
        super().__init__("minimal_beamformer_subscriber")
        self.get_logger().info("Setting up AcSense Beamformer Data Subscriber")
        self.subscription = self.create_subscription(
            AcSenseBeamformerData,
            "beamformer_data",
            self.listener_callback,
            10,  # CHANGE
        )

        self.plot = args.plot
        self.plot_init = False
        self.plot_history_cbf = deque([], maxlen=100)

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

        if self.plot:
            plot_data = np.squeeze(data - np.min(data), axis=2)
            plot_data /= np.max(plot_data)

            if not self.plot_init:
                plt.ion()
                self.fig, self.axs = plt.subplots(
                    2, 1, figsize=[12, 12], constrained_layout=True
                )
                self.ss1 = self.axs[0].pcolormesh(
                    np.rad2deg(msg.bearings), np.rad2deg(msg.elevations), plot_data.T
                )
                self.axs[0].set_xlabel("Bearing [deg]")
                self.axs[0].set_ylabel("Elevation [deg]")

                self.plot_history_cbf += deque(
                    np.zeros([self.plot_history_cbf.maxlen, len(msg.bearings)])
                )
                self.ss2 = self.axs[1].pcolormesh(
                    np.rad2deg(msg.bearings),
                    np.arange(self.plot_history_cbf.maxlen)
                    - self.plot_history_cbf.maxlen,
                    np.array(self.plot_history_cbf),
                )
                self.ss2.set_clim(self.ss1.get_clim())
                self.axs[1].set_xlabel("Bearing [deg]")
                self.axs[1].set_ylabel("Frame index")
                self.axs[1].sharex(self.axs[0])

                plt.show()
                self.plot_init = True
                pass

            self.plot_history_cbf.append(plot_data.mean(axis=1))

            self.ss1.set_array(plot_data.T)
            self.ss2.set_array(np.array(self.plot_history_cbf))
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        prog="AcSense Beamformer Subscriber",
        description="Captures data from the Acbotics Beamformer publisher in ROS, and optionally renders the data for testing",
        epilog="Need additional support? Contact Acbotics Research LLC (support@acbotics.com)",
    )
    parser.add_argument("--plot", action="store_true")
    args, unknown = parser.parse_known_args()

    minimal_subscriber = MinimalBeamformerRawSubscriber(args)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
