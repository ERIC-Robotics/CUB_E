FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl gnupg

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'

RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

RUN rosdep init && rosdep update

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install -y software-properties-common

RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update

RUN apt-get update && apt-get install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

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

WORKDIR /ros2_humble

RUN mkdir -p src \
    && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

RUN rosdep update && apt-get update \
    && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN colcon build --symlink-install

WORKDIR /ros2_depend

RUN mkdir -p src \
    && cd src \
    && git clone https://github.com/ros2/ros1_bridge.git 

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && source /ros2_humble/install/setup.bash \
    && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure'

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && source /ros2_humble/install/setup.bash \
    && source /ros2_depend/install/setup.bash'

CMD [ "bash" ]