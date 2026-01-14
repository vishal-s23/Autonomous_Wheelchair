# Autonomous Wheelchair Simulation (ROS 2 + Gazebo)

This project simulates a powered wheelchair using **ROS 2 Jazzy** and **Gazebo Harmonic**.

## Features
- Differential drive wheelchair
- Realistic wheel dynamics
- Gazebo DiffDrive plugin
- Keyboard teleoperation
- ROS–Gazebo bridge

## Requirements
- Ubuntu 22.04 / 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- ros_gz_sim
- teleop_twist_keyboard

## Build Instructions
```bash
cd ~/wheelchair_ws
colcon build
source install/setup.bash

Run Simulation
ros2 launch wheelchair_simulation spawn_wheelchair.launch.py

Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Controls

i → forward

, → backward

j → turn left

l → turn right

k → stop
