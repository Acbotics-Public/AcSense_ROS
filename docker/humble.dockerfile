FROM ros:humble-ros-base AS base

# For some reason, adding the linux/arm64 variant causes issues with libc-bin
# The patch below is based on the following Stack Overflow posts:
# https://stackoverflow.com/questions/78105004/docker-build-fails-because-unable-to-install-libc-bin
# https://stackoverflow.com/questions/73710118/trying-to-update-libc-bin-error-state-134-cant-sudo-install-net-tools/76260513#76260513
RUN rm /var/lib/dpkg/info/libc-bin.* \
    && apt clean \
    && apt update \
    && apt install libc-bin

RUN apt update \
    && apt upgrade -y

COPY ./ /acsense_ros
WORKDIR /acsense_ros
RUN rm -rf build install log

RUN /ros_entrypoint.sh ./build_ros_pkg.sh

RUN cp /acsense_ros/docker/ros_entrypoint.sh /


# Use ros-core in final stage for smaller image
FROM ros:humble-ros-core

COPY --from=base /acsense_ros/install /acsense_ros
COPY --from=base /ros_entrypoint.sh /ros_entrypoint.sh
