# Simple 2-DOF Robot Arm Model

## Overview

- For understanding what a robot arm is
- URDF example
- Example of displaying a robot model in RViz

## Installation

- Assume the ROS workspace is `~/airobot_ws`.
  ```
  cd ~/airobot_ws/src
  ```

- Get the repository including this package
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter6
  ```

- Build the package
  ```
  sudo apt install ros-humble-joint-state-publisher-gui
  cd ~/airobot_ws
  colcon build --packages-select simple_arm_description
  ```

## Execution

- Run the following in a terminal
  ```
  source install/setup.bash
  ros2 launch simple_arm_description display.launch.py
  ```
- Operate the sliders in the `joint_state_publisher_gui` window.

## Help

## Author

Yasuhiro Masutani

## History

- 2023-10-13: Operation confirmed on ROS Humble
- 2022-08-23: License and documentation organized

## License

Copyright (c) 2022-2025, MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## References
