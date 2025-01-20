# OAK-D Camera ROS 2 Launcher

![distro](https://img.shields.io/badge/Ubuntu%2024-Nobley%20Numbat-green)
![distro](https://img.shields.io/badge/ROS2-Jazzy-blue)
[![jazzy](https://github.com/jmguerreroh/oak_d_camera/actions/workflows/master.yaml/badge.svg?branch=jazzy)](https://github.com/jmguerreroh/oak_dcamera_ros2/actions/workflows/master.yaml)

This package provides a ROS 2 launch file to initialize and configure the OAK-D camera for RGBD, stereo, and point cloud processing. It includes adjustable camera configurations for transform frames, position, orientation, and depth processing parameters, along with a sample RViz configuration.

## Installation

1. Execute the below command to install the required dependencies:
    ```bash
    sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
    ```
    Please refer to [Installation documentation](https://docs.luxonis.com/software/depthai/manual-install#supported-platforms) if any issues occur.

2. Clone the repository and add it to your ROS 2 workspace:
    ```bash
    git clone https://github.com/jmguerreroh/oak_d_camera.git
    ```

3. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the workspace:
    ```bash
    colcon build --symlink-install
    ```

## Usage

To launch the OAK-D camera node with default parameters:

```bash
ros2 launch oak_d_camera rgbd_stereo.launch.py

```

## Launch Arguments

The launch file provides several configurable arguments:

#### Camera Parameters

- `camera_model`: Model of the camera. Options: `OAK-D`, `OAK-D-LITE`. Default: `OAK-D-LITE`
- `tf_prefix`: TF prefix for camera frames. Default: `oak`
- `base_frame`: Name of the camera's base frame. Default: `oak-d_frame`
- `parent_frame`: Parent frame for the camera's base. Default: `oak-d-base-frame`

#### Camera Position and Orientation

- `cam_pos_x`, `cam_pos_y`, `cam_pos_z`: Camera position relative to the base frame. Default: `0.0`
- `cam_roll`, `cam_pitch`, `cam_yaw`: Camera orientation relative to the base frame. Default: `0.0`

#### Depth Processing Parameters

- `lrcheck`: Enable left-right consistency check. Default: `True`
- `extended`: Enable extended disparity range. Default: `False`
- `subpixel`: Enable subpixel accuracy. Default: `True`
- `confidence`: Set confidence threshold for depth estimation. Default: `200`
- `LRchecktresh`: Set left-right consistency threshold. Default: `5`

Please refer to [StereoDepth documentation](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth) for more information.

#### Options

- `only_rgb`: Enable publishing of only RGB images. Default: `False`
- `use_rviz`: Launch RViz for visualization. Default: `True`
  `use_depth`: Enable depth image publishing. Default: `True`
- `use_disparity`: Enable disparity image publishing. Default: `True`
- `use_lr_raw`: Enable left and right raw image publishing. Default: `True`
- `use_pointcloud`: Enable point cloud publishing. Default: `True`
- `pc_color`: Use color in the point cloud. Options: `True` for color or `False` for intensity. Default: `True`
- `use_base`: Use the STL base bracket. Default: `True`
  
## Visualizing in RViz

The launch file opens RViz with a pre-configured view. You can modify `rviz/rgbd_stereo_pcl.rviz` to customize the visualization.

## About

This project was made by [José Miguel Guerrero], Associate Professor at [Universidad Rey Juan Carlos].

Copyright &copy; 2024.

[![Twitter](https://img.shields.io/badge/follow-@jm__guerrero-green.svg)](https://twitter.com/jm__guerrero)

## License

This work is licensed under the terms of the [MIT license](https://opensource.org/license/mit).

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero

