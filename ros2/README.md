# Cub_E - Autonomous Industrial Inspection Robot

**Cub_E** is an **autonomous robot** designed for industrial inspection tasks. It aims to reduce human effort by automating inspections and allowing remote monitoring of industrial environments.

## Prerequisites

Before using Cub_E, ensure you have the following dependencies installed:

- ROS2 `Humble` distribution
- Additional packages:
  - `gazebo*`
  - `xacro`
  - `rmw_cyclonedds_cpp`
  - `nav*`

>If you are using **`binary installation`**, you can install these packages using `apt`. 

>If you are using **`source installation`**, make sure you have built your ROS2 Humble source folder, and you'll need the `behaviours_tree_v3` package from the GitHub v3.8 branch. 

#### Build the repository using the following command:

```bash
colcon build --symlink-install
```

>Make sure you have sourced ROS2 Humble in your terminal before building.

## Installation

To get started, clone the repository:

```bash
git clone https://github.com/ERIC-Robotics/CUB_E.git
```

## Usage

  - Source your build workspace every time before running any files or nodes:

    ```bash
    source <path_to_your_workspace>/install/setup.bash
    ```

  - To launch the Gazebo simulation and spawn the robot with all plugins:

    ```bash
    ros2 launch cube_bringup gazebo_bringup.launch.py
    ```

  - For SLAM (Simultaneous Localization and Mapping) functionality:

    ```bash
    ros2 launch cube_navigation slam.launch.py
    ```

  - For launching navigation:

    ```bash
    ros2 launch cube_navigation navigation.launch.py
    ```

## Contents of directories:

- [**bno055**](https://github.com/ERIC-Robotics/CUB_E/tree/master/bno055): 

    >contains `launch` file and configuration files for `IMU`

- [**cube_bringup**](https://github.com/ERIC-Robotics/CUB_E/tree/master/cube_bringup):
      
    >contains `launch` files for bring up all necessary nodes

- [**cube_description**](https://github.com/ERIC-Robotics/CUB_E/tree/master/cube_description):

    >contains `Urdf` files and simulation `plugins` and materials

- [**cube_gazebo**](https://github.com/ERIC-Robotics/CUB_E/tree/master/cube_gazebo):

    >conatins `launch` files for bringing up `gazebo` 

- [**cube_hardware**](https://github.com/ERIC-Robotics/CUB_E/tree/master/cube_hardware):

    >conatins `launch` files for bring up all hardware related nodes and has `exectuables` scripts in `src`

- [**cube_navigation**](https://github.com/ERIC-Robotics/CUB_E/tree/master/cube_navigation):

    >contains `launch` files and parameter `config` files for `navigationa stack` and `slam`

- [**YDLidar**](https://github.com/ERIC-Robotics/CUB_E/tree/master/YDLidar):

    >contains `launch` file and configuration files for `lidar`


## Contributing

Contributions to this project are not currently accepted, as it is a private repository managed by `Eric Robotics`, `Premier Seals PVT LTD`.


## License

>This project is not open-source and is a private repository.