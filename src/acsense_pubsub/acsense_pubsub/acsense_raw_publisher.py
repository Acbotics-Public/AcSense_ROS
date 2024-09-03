import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from acsense_ros_interfaces.msg import (
    AcSenseRawData,
    AcSenseRawDataSingleChannel,
)

import socket
import struct
import logging

import argparse
import traceback

from acbotics_interface.protocols.udp_data_protocol import UDP_Data_Protocol


logger = logging.getLogger(__name__)


class AcSenseRawPublisher(Node):
    def __init__(self, args):
        super().__init__("minimal_ac_publisher")
        self.publisher_ = self.create_publisher(AcSenseRawData, "raw_data", 10)

        logger.info(f"Setting up AcSense Raw Publisher")

        self.bot = UDP_Data_Protocol()

        self.sock_aco = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )
        self.sock_aco.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if not args.use_mcast:
            self.sock_aco.bind((args.iface_ip, args.port))

        else:
            self.sock_aco.bind(("", args.port))

            group = socket.inet_aton(args.mcast_group)
            mreq = struct.pack("4s4s", group, socket.inet_aton(args.iface_ip))
            self.sock_aco.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        logger.info(f"Launching AcSense Raw Publisher")
        self.run()

    def run(self):
        while True:
            try:
                msg = self.sock_aco.recv(65535)

                new_header = self.bot.decode_header(msg)
                new_data = self.bot.decode(msg).data

                msg = AcSenseRawData()
                msg.ver_maj = new_header.VER_MAJ
                msg.ver_min = new_header.VER_MIN

                msg.num_channels = new_header.NUM_CHANNELS
                msg.data_size = new_header.DATA_SIZE
                msg.num_values = new_header.NUM_VALUES
                if type(new_header.SAMPLE_RATE) == int:
                    msg.sample_rate_int = new_header.SAMPLE_RATE
                else:
                    msg.sample_rate_float = new_header.SAMPLE_RATE

                msg.start_time = new_header.START_TIME
                msg.adc_count = new_header.ADC_COUNT
                msg.scale = new_header.SCALE
                msg.packet_num = new_header.PACKET_NUM

                array_data = []
                for ch in range(new_header.NUM_CHANNELS):
                    ch_data = AcSenseRawDataSingleChannel()
                    ch_data.channel_data = new_data[ch, :].tolist()
                    array_data.append(ch_data)

                msg.array_data = array_data

                self.publisher_.publish(msg)

                self.get_logger().info(f"Publishing: {new_header.PACKET_NUM}")

                # self.get_logger().info(
                #     f"Publishing: {new_header.PACKET_NUM}, {new_header.NUM_CHANNELS} ch x {len(ch_data.channel_data)} ({new_header.NUM_VALUES}) values"
                # )

                # self.get_logger().info(f"{new_header.NUM_CHANNELS}")
                # self.get_logger().info(f"{new_header.DATA_SIZE}")
                # self.get_logger().info(f"{new_header.NUM_VALUES}")
                # self.get_logger().info(f"{new_header.SAMPLE_RATE}")
                # self.get_logger().info(f"{new_header.START_TIME}")

            except (TimeoutError, socket.timeout) as e:
                logger.debug(e)
                continue
            except Exception as e:
                logger.error(e)
                traceback.print_exc()
                continue


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        prog="AcSense Publisher",
        description="Captures UDP data from the AcSense and publishes it into ROS",
        epilog="Need additional support? Contact Acbotics Research LLC (support@acbotics.com)",
    )
    parser.add_argument("--use-mcast", action="store_true")
    parser.add_argument("--mcast-group", default="224.1.1.1")
    parser.add_argument("--iface-ip", default="192.168.1.115")
    parser.add_argument("--port", type=int, default=9760)
    parser.add_argument("--debug", action="store_true")

    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    logging.basicConfig(
        # format="[%(asctime)s] %(name)s.%(funcName)s() : \n\t%(message)s",
        format="[%(asctime)s] %(levelname)s: %(filename)s:L%(lineno)d : %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        level=logging.DEBUG if args.debug else logging.INFO,
        force=True,
    )

    acsense_publisher = AcSenseRawPublisher(args)

    rclpy.spin(acsense_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    acsense_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
