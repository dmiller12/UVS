FROM ros_noetic:latest

SHELL ["/bin/bash", "-c"]


RUN apt-get update; apt-get install -y libopencv-dev python3-opencv

#RUN mkdir -p /opt/opencv && cd /opt/opencv
#RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.7.0.zip; \
#unzip opencv.zip; \
#wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.7.0.zip; \
#unzip opencv_contrib.zip

#RUN mkdir -p build && cd build

#RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.7.0/modules ../opencv-4.7.0

#RUN cmake --build . -j 4 --target install

# other 3rd party dependencies
RUN apt-get install -y libeigen3-dev


#ros dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-usb-cam


RUN mkdir -p /root/catkin_ws/src
COPY uvs_bridge  /root/catkin_ws/src/uvs_bridge
COPY wam_common  /root/catkin_ws/src/wam_common

RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash && cd /root/catkin_ws && catkin_make && source devel/setup.bash'

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
