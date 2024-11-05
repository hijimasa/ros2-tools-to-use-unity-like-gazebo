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

7. Publish JointState
   
   Example
   ```
   ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: { stamp: { sec: 0, nanosec: 0}, frame_id: ''}, name: ["right_wheel_joint", "left_wheel_joint"], position: [], velocity: [10.0, 10.0], effort: []}" 
   ```
