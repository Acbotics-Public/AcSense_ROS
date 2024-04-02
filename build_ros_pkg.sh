#!/usr/bin/env bash

# Get non-rosdep dependencies
git submodule init
git submodule update

# Get rosdep dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Run colcon build
colcon build --packages-select acsense_ros_interfaces
colcon build --packages-select acsense_pubsub

echo ""
echo "===================="
echo ""
echo "To run the provided publisher and listener, run setup.bash and use the following 'ros2 run' commands"
echo ""
echo "  source ./install/setup.bash"
echo ""
echo "  ros2 run acsense_pubsub publisher"
echo "  ros2 run acsense_pubsub listener"
