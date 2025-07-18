# Simple Node Using ROS 2 Nodes for CRANE+ V2

## Overview

- A node that uses the [crane_plus](https://github.com/rt-net/crane_plus) ROS 2 nodes for the CRANE+ V2 robot arm, published by RT Corporation.
- Supports both real hardware and simulation.
- All node programs are written in Python.
- Includes both non-MoveIt command-style and MoveIt-based control methods.
- Uses [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) as the Python interface to MoveIt 2.
- Uses a [forked version](https://github.com/AI-Robot-Book-En/crane_plus) of crane_plus instead of the [original by RT Corporation](https://github.com/rt-net/crane_plus).
- Developed and tested on Ubuntu 22.04 with ROS Humble.

## Preparation

- If using the real CRANE+ V2, follow the instructions in the [crane_plus_control README](https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/README.md).  
  - Key points:
    - 1. Configure USB communication port (e.g., `sudo chmod 666 /dev/ttyUSB0`)
    - 2. Modify USB latency_timer settings
    - 3. Configure Return Delay Time for each actuator of the robot
  - For 1 and 2, you can add a configuration file under `/etc/udev/rules.d` to automatically apply the settings when connecting the USB device  
    (see [ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#copy-rules-file) for details).
  - If using the book's Docker image with a Linux host, configure and verify the CRANE+ V2 connection on the host.

## Installation

- Assume the ROS workspace is `~/airobot_ws`.
  ```
  cd ~/airobot_ws/src
  ```

- Clone the forked version of crane_plus (not RT Corporation’s version):
  ```
  git clone https://github.com/AI-Robot-Book-En/crane_plus
  ```

- Follow the instructions in the [crane_plus README](https://github.com/AI-Robot-Book-En/crane_plus/blob/master/README.md):
  ```
  rosdep install -r -y -i --from-paths .
  cd ~/airobot_ws
  colcon build
  source install/setup.bash
  ```

- Get and build the [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) package:
  ```
  cd ~/airobot_ws/src
  git clone https://github.com/AndrejOrsula/pymoveit2
  rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
  cd ~/airobot_ws
  colcon build --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
  source install/setup.bash
  ```

- Clone the repository containing this package:
  ```
  cd ~/airobot_ws/src
  git clone https://github.com/AI-Robot-Book-En/chapter6
  ```

- Clone the repository containing the action interface definitions:
  ```
  git clone https://github.com/AI-Robot-Book-En/chapter2
  ```

- Clone the repository with the test client for action communication:
  ```
  git clone https://github.com/AI-Robot-Book-En/appendixB
  ```

- Build the packages:
  ```
  cd ~/airobot_ws
  colcon build --packages-select airobot_interfaces crane_plus_commander
  source install/setup.bash
  ```

## Execution (Without MoveIt)

- Terminal 1:
  - Set overlay:
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - For real hardware:
    ```
    ros2 launch crane_plus_examples no_moveit_demo.launch.py
    ```

  - Using Ignition Gazebo instead of real hardware:
    ```
    ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py 
    ```

- Terminal 2:
  - Set overlay:
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - Change joint angles using keyboard:
    ```
    ros2 run crane_plus_commander commander1
    ```

  - Change end-effector position using keyboard:
    ```
    ros2 run crane_plus_commander commander2
    ```

  - Change joint angles and display joint states:
    ```
    ros2 run crane_plus_commander commander3
    ```

  - Synchronous action client (waits for result):
    ```
    ros2 run crane_plus_commander commander4
    ```

  - Move end-effector to a point given in tf frame:
    ```
    ros2 run crane_plus_commander commander5
    ```

## Execution (With MoveIt)

- Terminal 1:
  - Set overlay:
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - For real hardware:
    ```
    ros2 launch crane_plus_examples endtip_demo.launch.py 
    ```

  - Using Ignition Gazebo instead of real hardware:
    ```
    ros2 launch crane_plus_gazebo endtip_crane_plus_with_table.launch.py 
    ```

- Terminal 2:
  - Set overlay:
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - Change end-effector position using keyboard (using MoveIt):
    ```
    ros2 run crane_plus_commander commander2_moveit
    ```

  - Move end-effector to a point in tf frame (using MoveIt):
    ```
    ros2 run crane_plus_commander commander5_moveit
    ```

  - Use as an action server (using MoveIt):
    ```
    ros2 run crane_plus_commander commander6_moveit
    ```

- Terminal 3 (for testing the action server):
  - Test client:
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ros2 run airobot_action test_client /manipulation/command
    ```

## Help

## Author

Yasuhiro Masutani

## History

- 2024-09-15: MoveIt integration added
- 2023-10-15: Compatible with ROS Humble
- 2022-08-23: License and documentation organized

## License

Copyright (c) 2022-2025, MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## References

- https://github.com/rt-net/crane_plus
- https://github.com/AndrejOrsula/pymoveit2
