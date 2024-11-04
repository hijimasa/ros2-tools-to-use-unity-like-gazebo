# ros2-tools-to-use-unity-like-gazebo
This repository is under development.

## How to Use

1. launch Unity and file server.
   ```bash
   launch unity_diffbot_sim bringup_unity_environment.launch.py
   ```
   
2. Click on the Unity window to activate the script

3. Import URDF.
   ```
   ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
   ```
   
