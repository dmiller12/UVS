FROM dylanjmiller/ros_noetic:latest

SHELL ["/bin/bash", "-c"]


RUN apt-get update; apt-get install -y libopencv-dev python3-opencv valgrind

# other 3rd party dependencies

RUN wget "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip" && \
    unzip eigen-3.4.0.zip
RUN cd eigen-3.4.0 && mkdir build && cd build && cmake .. && make install
RUN rm -rf eigen*

#ros dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-usb-cam


RUN mkdir -p /root/catkin_ws/src
COPY uvs_bridge  /root/catkin_ws/src/uvs_bridge
COPY wam_common  /root/catkin_ws/src/wam_common

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /root/catkin_ws && catkin_make && source devel/setup.bash

WORKDIR /root/catkin_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
