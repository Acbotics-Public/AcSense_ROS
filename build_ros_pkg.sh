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
