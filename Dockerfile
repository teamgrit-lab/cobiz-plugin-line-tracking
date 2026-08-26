ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-setuptools \
        ros-humble-cv-bridge \
        ros-humble-geometry-msgs \
        ros-humble-launch-ros \
        ros-humble-nav-msgs \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-sensor-msgs \
        ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src ./src

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
