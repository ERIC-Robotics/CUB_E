FROM ros:noetic

RUN apt-get update && apt-get install -y \
    ros-noetic-rosserial* \
    ros-noetic-cv-bridge* \
    ros-noetic-rospy*

RUN apt-get update && apt-get install -y \
    ros-noetic-pcl-ros*

RUN apt-get install -y libudev-dev
    
COPY /ros1/src /catkin_ws/src

WORKDIR /catkin_ws

RUN apt-get update && cd src/BMVM-OS30A-ROS-SDK/HD-DM-Linux-SDK-5.0.1.17/DMPreview \
    && sh setup_env.sh

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && catkin_make'

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD bash