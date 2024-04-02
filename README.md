# ROS2 driver for AcSense

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

This package provides a network driver for the AcSense. Both directed UDP and multicast UDP configurations are available.

**NOTE: ROS 1 version is provided under the `noetic` branch**

## Installation

To build and use the package, call the provided `build_ros_pkg.sh` script from your ROS environment.

## Usage

The following Python `argparse` parameters are available for the publisher node:

- `--use-mcast` : logical flag indicates that the multicast socket configuration shall be used
- `--mcast-group` : sets the value for the UDP multicast group (default: 224.1.1.1)
- `--iface-ip` : sets the IP address of the target interface (on the host computer; default: 192.168.1.115)
- `--port` : sets the port on which to receive the data, for either direct or multicast UDP configuration
