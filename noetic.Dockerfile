FROM ros:noetic

RUN apt-get update && apt-get install -y \
    ros-noetic-rosserial* \
    ros-noetic-cv-bridge* 

RUN apt-get install -y libudev-dev

RUN apt-get update && apt-get install -y \
    ros-noetic-rospy*
    
COPY ros1 /catkin_ws/src

WORKDIR /catkin_ws

RUN cd src/BMVM-OS30A-ROS-SDK/HD-DM-Linux-SDK-5.0.1.17/DMPreview \
    && sh setup_env.sh

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && catkin_make'

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD bash