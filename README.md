# Chapter 6: Manipulation

## Overview

This repository contains sample programs and supplementary information for Chapter 6 of a certain book.

## Directory Structure

- [crane_plus_commander](crane_plus_commander): A simple set of ROS 2 nodes for controlling the CRANE+ V2

- [simple_arm/simple_arm_description](simple_arm/simple_arm_description): A model of a simple 2-DOF robot arm

## Sample Program List

- Program Listing 6.1: [Part of kinematics.py](crane_plus_commander/crane_plus_commander/kinematics.py#L59-L68)
- Program Listing 6.2: [Part of kinematics.py](crane_plus_commander/crane_plus_commander/kinematics.py#L71-L95)
- Program Listing 6.2: [simple_arm.urdf](simple_arm/simple_arm_description/urdf/simple_arm.urdf)
- Section 6.5.5: Program to move joints [commander1.py](crane_plus_commander/crane_plus_commander/commander1.py)
- Section 6.5.6: Program to move the end effector [commander2.py](crane_plus_commander/crane_plus_commander/commander2.py)
- Section 6.5.7: Program to receive the robot's state [commander3.py](crane_plus_commander/crane_plus_commander/commander3.py)
- Section 6.5.8: Program using ROS 2 Action communication [commander4.py](crane_plus_commander/crane_plus_commander/commander4.py)
- Section 6.6.3: Program using tf [commander5.py](crane_plus_commander/crane_plus_commander/commander5.py)
- Section 6.7.4: Program for kinematic calculation using MoveIt [commander2_moveit.py](crane_plus_commander/crane_plus_commander/commander2_moveit.py)
- Section 6.7.5: Program to move the end effector using MoveIt [commander5_moveit.py](crane_plus_commander/crane_plus_commander/commander5_moveit.py)
- Section 6.8: Program to operate based on commands from another node [commander6_moveit.py](crane_plus_commander/crane_plus_commander/commander6_moveit.py)

## Additional Information

- [Preparation for using the actual CRANE+ V2 robot](crane_plus_commander#準備) section has been added. (2022/12/7)
