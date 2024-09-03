#!/usr/bin/env bash

# Get non-rosdep dependencies
git submodule init
git submodule update

ROOTDIR=$(dirname $0)
echo "Root dir is : $ROOTDIR"
if [ -d $ROOTDIR/src/acsense_pubsub/acbotics_interface ]; then
    rm -rf $ROOTDIR/src/acsense_pubsub/acbotics_interface
fi
if [ -d $ROOTDIR/src/acsense_pubsub/icontract ]; then
    rm -rf $ROOTDIR/src/acsense_pubsub/icontract
fi
if [ -d $ROOTDIR/src/acsense_pubsub/asttokens ]; then
    rm -rf $ROOTDIR/src/acsense_pubsub/asttokens
fi
cp -r $ROOTDIR/submodules/icontract/icontract $ROOTDIR/src/acsense_pubsub/
cp -r $ROOTDIR/submodules/asttokens/asttokens $ROOTDIR/src/acsense_pubsub/
cp -r $ROOTDIR/submodules/acbotics_interface/src/acbotics_interface $ROOTDIR/src/acsense_pubsub/

# Get rosdep dependencies
# rosdep install -i --from-path src --rosdistro humble -y
rosdep install -i --from-path src -y

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
echo "  ros2 run acsense_pubsub ac_publisher"
echo "  ros2 run acsense_pubsub ac_subscriber"
echo ""
echo "  ros2 run acsense_pubsub beamform_raw_publisher"
echo "  ros2 run acsense_pubsub beamform_raw_subscriber"
