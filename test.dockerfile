# Stage 1: Create a Noetic container
FROM osrf/ros:noetic-desktop-full AS noetic
# RUN apt-get update && apt-get install -y \
#     # Install additional Noetic packages if needed

# Stage 2: Create a Foxy container
FROM osrf/ros:foxy-ros1-bridge AS foxy

# Stage 3: Final image combining Noetic and Foxy
FROM ubuntu:20.04
LABEL maintainer="Your Name <your@email.com>"

# Copy Noetic and Foxy installations from the previous stages
COPY --from=noetic /opt/ros/noetic /opt/ros/noetic
COPY --from=foxy /opt/ros/foxy /opt/ros/foxy

# Set up environment variables
# ENV ROS_DISTRO noetic
# ENV ROS_ROOT /opt/ros/noetic
# ENV ROS_PACKAGE_PATH /opt/ros/noetic/share
# ENV ROS_MASTER_URI http://localhost:11311
# ENV ROS_VERSION 1

RUN apt-get update && apt-get install -y \
  python3 \
  python3-pip 

# Source the ROS setup file
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

CMD [ "bash" ]