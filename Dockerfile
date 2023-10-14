FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl gnupg git cmake 

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-ompl* \
    ros-noetic-rosserial* \
    ros-noetic-cv-bridge* 

RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    libzmq3-dev \
    libboost-dev \
    libgeographic-dev \
    graphicsmagick \
    libgraphicsmagick++1-dev \
    libcholmod3 \
    libceres-dev \
    libsuitesparse-dev \
    libudev-dev

RUN rosdep init && rosdep update

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install -y software-properties-common

RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    python3 \
    python3-pip

RUN python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    "pytest>=5.3" \
    pytest-repeat \
    pytest-rerunfailures

RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git --branch v3.8\
    && cd BehaviorTree.CPP \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN git clone https://github.com/ompl/ompl.git --branch 1.5.2 \
    && cd ompl \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds.git --branch releases/0.10.x \
    && cd cyclonedds \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

WORKDIR /ros2_humble

RUN mkdir -p src \
    && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src 

RUN rosdep update && apt-get update \
    && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN colcon build --symlink-install

WORKDIR /ros2_depend

RUN mkdir -p src \
    && cd src \
    && git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git --branch ros2 \
    && git clone https://github.com/ros/xacro.git --branch ros2 \
    && git clone https://github.com/ros2/rmw_cyclonedds.git --branch humble \
    && git clone https://github.com/ros2/ros1_bridge.git \
    && git clone https://github.com/cra-ros-pkg/robot_localization.git --branch humble-devel \
    && git clone https://github.com/ros-planning/navigation2.git --branch humble \
    && git clone https://github.com/ros/angles.git --branch humble-devel \
    && git clone https://github.com/ros-perception/vision_opencv.git --branch humble \
    && git clone https://github.com/SteveMacenski/slam_toolbox.git --branch humble \
    && git clone https://github.com/ros/bond_core.git --branch ros2 \
    && git clone https://github.com/eclipse-iceoryx/iceoryx.git --branch release_2.0 \
    && git clone https://github.com/ros/diagnostics.git --branch ros2 \
    && git clone https://github.com/ros-geographic-info/geographic_info.git --branch ros2

RUN cd src/navigation2 && rm -rf nav2_mppi* nav2_smac*

RUN cd src/gazebo_ros_pkgs && rm -rf gazebo_plugins

RUN /bin/bash -c 'source /ros2_humble/install/setup.bash \
    && colcon build --symlink-install --parallel-workers 2 --packages-skip ros1_bridge'

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && source /ros2_humble/install/setup.bash \
    && source /ros2_depend/install/setup.bash \
    && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure'

COPY /ros2 /colcon_ws/src

WORKDIR /colcon_ws

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && source /ros2_humble/install/setup.bash \
    && source /ros2_depend/install/setup.bash \
    && colcon build --symlink-install'

COPY /ros1 /catkin_ws/src

WORKDIR /catkin_ws

RUN cd src/BMVM-OS30A-ROS-SDK/HD-DM-Linux-SDK-5.0.1.17/DMPreview \
    && sh setup_env.sh

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && catkin_make'

CMD ["bash"]
