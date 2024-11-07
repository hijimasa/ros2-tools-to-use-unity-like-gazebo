# ros2-tools-to-use-unity-like-gazebo
English | [日本語](README_ja.md)
![demo](./figs/ros2_unity_demo.gif)

This repository shows how to control the robot from ros2_control to make Unity easier to use.
"unity_ros2_scripts" has the python script to launch and control Unity.

The features of this repository are below:
- This shows how to control a robot on Unity with ros2_control.
- This provides a Dockerfile where Unity and ROS2 Humble can coexist.
- This currently supports prismatic and rotational joints using position and velocity control.
- This sends joint status (position, velocity and effort) to ros2_control from Unity.
- This spawns URDF model at the desired timing and position.
- This sets stiffness, damping and friction from URDF description.

## Prerequisite
1. Docker
1. Unity account

## Prepare Docker container
1. Clone this repository
   ```
   git clone https://github.com/hijimasa/ros2-tools-to-use-unity-like-gazebo.git
   ```
2. Initialize Git submodule
   ```
   cd ros2-tools-to-use-unity-like-gazebo/
   git submodule update --init --recursive
   ```
3. Move to docker directory
   ```
   cd docker
   ```
4. Build docker image
   ```
   ./build-dokcer-image.bash
   ```
5. Run docker container
   ```
   ./run-docker-container.bash
   ```
6. Run UnityHub
   ```
   unityhub
   ```
7. Sign in Unity
   
## How to Use

Tips: Use below command to launch another docker terminal
```
docker exec -it ros-humble-unity /bin/bash
```

1. launch Unity.
   ```bash
   ros2 run unity_ros2_scripts launcher
   ```
   
2. Click on the Unity window to activate the script

3. Import URDF.
   ```
   ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
   ```

4. launch ros_tcp_endpoint
   ```
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

5. Add "3D Object Plane" to use ground plane.

6. Run Simulation

7. launch teleop_twist_keyboard
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
