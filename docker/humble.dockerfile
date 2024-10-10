FROM ros:humble-ros-base
SHELL ["bash","-c"]

RUN apt update \
    && apt upgrade -y

# RUN apt install -y \
#     git

COPY ./ /acsense_ros
WORKDIR /acsense_ros
RUN rm -rf build install log

RUN /ros_entrypoint.sh ./build_ros_pkg.sh

RUN cp /acsense_ros/docker/ros_entrypoint.sh /