ARG TAG=latest
FROM ast_interfaces:${TAG} as ast_inter
FROM ast_utils:${TAG} as ast_utils
FROM ros:humble

ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2_ws

WORKDIR $WORKSPACE

RUN apt-get update -qq && \
    apt-get upgrade -qq -y --with-new-pkgs && \
    apt-get install -qq -y \
      ccache \
      lcov \
      lld \
      python3-pip \
      ros-$ROS_DISTRO-rmw-fastrtps-cpp \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
      ros-$ROS_DISTRO-sensor-msgs \
      ros-$ROS_DISTRO-geometry-msgs \
      ros-$ROS_DISTRO-joy \
    && pip3 install \
      fastcov \
      rosdep \
      git+https://github.com/ruffsl/colcon-cache.git \
      git+https://github.com/ruffsl/colcon-clean.git \
    && rosdep update \
    && colcon mixin update \
    && colcon metadata update \
    && rm -rf /var/lib/apt/lists/*

COPY ./ ./src/ast_joystick
COPY --from=ast_inter /ros2_ws/src/ast_interfaces /ros2_ws/src/ast_interfaces
COPY --from=ast_utils /ros2_ws/src/ast_utils /ros2_ws/src/ast_utils

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon cache lock && \
    colcon build \
      --symlink-install \
      --mixin release ccache lld \
    && colcon test \
      --event-handlers console_direct+

# source overlay from entrypoint
ENV WORKSPACE $WORKSPACE
RUN sed --in-place \
      's|^source .*|source "$WORKSPACE/install/setup.bash"|' \
      /ros_entrypoint.sh