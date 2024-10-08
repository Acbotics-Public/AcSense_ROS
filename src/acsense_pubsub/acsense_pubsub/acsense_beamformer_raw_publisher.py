import argparse
import logging
import socket
import struct
import traceback

import rclpy  # type: ignore
from acbotics_interface.protocols.udp_beamform_raw_protocol import (
    UDP_Beamform_Raw_Protocol,
)
from rclpy.node import Node  # type: ignore

from acsense_ros_interfaces.msg import (  # type: ignore
    AcSenseBeamformerData,
)


class AcSenseBeamformRawPublisher(Node):
    def __init__(self, args):
        super().__init__("minimal_beamformer_raw_publisher")
        self.publisher_ = self.create_publisher(
            AcSenseBeamformerData, "beamformer_data", 1
        )

        self.get_logger().info("Setting up AcSense Beamformer:Raw Publisher")

        self.bot = UDP_Beamform_Raw_Protocol()

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

        self.get_logger().info("Launching AcSense Beamformer:Raw Publisher")
        self.run()

    def run(self):
        new_header = None
        payload_frames = {}
        while True:
            try:
                msg = self.sock_aco.recv(65535)
                if msg[:4] == b"ACBR":
                    new_header = self.bot.decode_header(msg)
                    payload_frames = {0: msg}

                elif msg[:4] == b"ACBC":
                    new_cont_header = self.bot.decode_continued_header(msg)
                    if (
                        new_header
                        and new_cont_header
                        and new_cont_header.PACKET_NUM == new_header.PACKET_NUM
                    ):
                        payload_frames.update(
                            {
                                new_cont_header.SUB_PACKET_INDEX: self.bot.get_continued_payload(
                                    msg
                                )
                            }
                        )
                    else:
                        payload_frames = {}
                        new_header = None

                if new_header and len(payload_frames) == new_header.NUM_PACKETS:
                    msg = b"".join(
                        [payload_frames[key] for key in range(new_header.NUM_PACKETS)]
                    )

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
        prog="AcSense Beamformer:Raw Publisher",
        description="Captures UDP data from the Acbotics Beamformer and publishes it into ROS",
        epilog="Need additional support? Contact Acbotics Research LLC (support@acbotics.com)",
    )
    parser.add_argument("--use_mcast", action="store_true")
    parser.add_argument("--mcast_group", default="224.1.1.1")
    parser.add_argument("--iface_ip", default="127.0.0.1")
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

    acsense_publisher = AcSenseBeamformRawPublisher(args)

    rclpy.spin(acsense_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    acsense_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
