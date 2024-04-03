#!/usr/bin/env python3

import rospy

import sys
import rospkg

pkg_path = rospkg.RosPack().get_path("acsense_pubsub")
sys.path.append(pkg_path)

from acsense_ros_interfaces.msg import AcSenseRawData


class MinimalSubscriber(object):
    """Minimal sample subscriber for AcSense acsense_raw_audio"""

    def __init__(self):
        super().__init__()
        self.subscription = rospy.Subscriber(
            "acsense_raw_audio", AcSenseRawData, self.listener_callback
        )

    def listener_callback(self, msg):
        """Sample callback verifies successful parsing of acsense_raw_audio"""
        rospy.loginfo(
            (
                f"Received packet no. {msg.packet_num} with {len(msg.array_data)} ch "
                f"x {len(msg.array_data[0].channel_data)} samples per ch"
            )
        )


def main(args=None):
    """Entrypoint for AcSense sample subscriber"""

    rospy.init_node("acsense_subscriber")

    minimal_subscriber = MinimalSubscriber()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
