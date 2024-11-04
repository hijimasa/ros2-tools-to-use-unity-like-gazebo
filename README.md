# ros2-tools-to-use-unity-like-gazebo
This repository is under development.

## Prepare Docker container
1. Clone this repository
   ```
   git clone https://github.com/hijimasa/ros2-tools-to-use-unity-like-gazebo.git
   ```
2. Move to docker directory
   ```
   cd ros2-tools-to-use-unity-like-gazebo/docker
   ```
3. Build docker image
   ```
   ./build-dokcer-image.bash
   ```
4. Run docker container
   ```
   ./run-docker-container.bash
   ```
   
## How to Use

1. launch Unity and file server.
   ```bash
   ros2 launch unity_diffbot_sim bringup_unity_environment.launch.py
   ```
   
2. Click on the Unity window to activate the script

3. Import URDF.
   ```
   ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
   ```
   
