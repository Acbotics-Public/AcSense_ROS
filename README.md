# ROS1 driver for AcSense

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

This package provides a network driver for the AcSense. Both directed UDP and multicast UDP configurations are available.

**NOTE: ROS 2 version is provided under the `main` branch**

## Installation

To build and use the package, call the provided `build_ros_pkg.sh` script from your ROS environment.

## Usage

The publisher's `roslaunch` argument `udp_args` can be used to provide the following Python `argparse` parameters for the network socket configuration:

- `--use-mcast` : logical flag indicates that the multicast socket configuration shall be used
- `--mcast-group` : sets the value for the UDP multicast group (default: 224.1.1.1)
- `--iface-ip` : sets the IP address of the target interface (on the host computer; default: 192.168.1.115)
- `--port` : sets the port on which to receive the data, for either direct or multicast UDP configuration

The following entry-points are provided for use with ros2 run (example uses multicast UDP argument):

```
ros2 run acsense_pubsub publisher --use-mcast
ros2 run acsense_pubsub listener
```