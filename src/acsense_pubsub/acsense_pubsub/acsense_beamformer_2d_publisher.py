import rclpy
from rclpy.node import Node

from acsense_ros_interfaces.msg import (
    AcSenseBeamformerData,
)

import socket
import struct
import logging

import argparse
import traceback

from acbotics_interface.protocols.udp_beamform_2d_protocol import (
    UDP_Beamform_2D_Protocol,
)


class AcSenseBeamform2DPublisher(Node):
    def __init__(self, args):
        super().__init__("minimal_beamformer_2d_publisher")
        self.publisher_ = self.create_publisher(
            AcSenseBeamformerData, "beamformer_data", 1
        )

        self.get_logger().info(f"Setting up AcSense Beamformer:2D Publisher")

        self.bot = UDP_Beamform_2D_Protocol()

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

        self.get_logger().info(f"Launching AcSense Beamformer:2D Publisher")
        self.run()

    def run(self):
        new_header = None
        while True:
            try:
                msg = self.sock_aco.recv(65535)
                if msg[:4] == b"ACB2":
                    new_header = self.bot.decode_header(msg)

                    dc = self.bot.decode(msg)

                    msg = AcSenseBeamformerData()

                    msg.ver_maj = new_header.VER_MAJ
                    msg.ver_min = new_header.VER_MIN

                    msg.num_elements = new_header.NUM_ELEMENTS
                    msg.num_frequencies = new_header.NUM_FREQUENCIES
                    msg.num_bearings = new_header.NUM_THETAS
                    msg.num_elevations = new_header.NUM_PHIS

                    msg.sample_rate = new_header.SAMPLE_RATE
                    msg.window_length_s = new_header.WINDOW_LENGTH_S

                    msg.start_time = new_header.START_TIME

                    msg.xform_pitch = new_header.XFORM_PITCH
                    msg.xform_roll = new_header.XFORM_ROLL
                    msg.xform_yaw = new_header.XFORM_YAW

                    msg.mode = ord(new_header.MODE)
                    msg.weighting_type = ord(new_header.WEIGHTING_TYPE)
                    msg.packet_num = new_header.PACKET_NUM

                    msg.element_x = dc.array_x.tolist()
                    msg.element_y = dc.array_y.tolist()
                    msg.element_z = dc.array_z.tolist()

                    msg.beamformer_freq = dc.frequencies.tolist()
                    msg.element_mask = dc.element_mask.tolist()
                    msg.element_weights = dc.element_weights.tolist()

                    msg.bearings = dc.thetas.tolist()
                    msg.elevations = dc.phis.tolist()
                    msg.beamformer_response = dc.data.flatten().tolist()
                    msg.shape = dc.data.shape

                    self.publisher_.publish(msg)

                    self.get_logger().info(f"Publishing: {new_header.PACKET_NUM}")

            except (TimeoutError, socket.timeout) as e:
                self.get_logger().debug(e)
                continue
            except Exception as e:
                self.get_logger().error(e)
                traceback.print_exc()
                continue


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        prog="AcSense Beamformer:2D Publisher",
        description="Captures UDP data from the Acbotics Beamformer and publishes it into ROS",
        epilog="Need additional support? Contact Acbotics Research LLC (support@acbotics.com)",
    )
    parser.add_argument("--use-mcast", action="store_true")
    parser.add_argument("--mcast-group", default="224.1.1.1")
    parser.add_argument("--iface-ip", default="192.168.1.115")
    parser.add_argument("--port", type=int, default=9765)
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

    acsense_publisher = AcSenseBeamform2DPublisher(args)

    rclpy.spin(acsense_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    acsense_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
