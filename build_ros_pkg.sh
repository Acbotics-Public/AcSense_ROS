#!/usr/bin/env bash

# Get non-rosdep dependencies
git submodule init
git submodule update

# Get rosdep dependencies
rosdep install -i --from-path src --rosdistro noetic -y

# Run catkin_make
catkin_make

echo ""
echo "===================="
echo ""
echo "To run the provided publisher and listener, run setup.bash and use the following 'roslaunch' commands"
echo ""
echo "  source ./devel/setup.bash"
echo ""
echo "  roslaunch acsense_pubsub acsense.launch"
