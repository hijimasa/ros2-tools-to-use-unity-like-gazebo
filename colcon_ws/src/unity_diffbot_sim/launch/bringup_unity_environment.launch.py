# © Siemens AG, 2024
# Author: Mehmet Emre Cakal (emre.cakal@siemens.com)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    port_arg = DeclareLaunchArgument('port', default_value='9090', description='Port number for ROS communication')

    # Define paths
    pkg_file_server2 = get_package_share_directory('file_server2')
    rosbridge_server_launch_file = PathJoinSubstitution(
        [pkg_file_server2, 'launch', 'ros_sharp_communication.launch.py'])
        
    # Include ROS Sharp Communication launch file
    ros_sharp_communication_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbridge_server_launch_file]),
                launch_arguments=[('port', LaunchConfiguration('port'))]
        )
    
    node_unity_launcher = Node(
        package='unity_ros2_scripts',
        executable='launcher',
        output='screen',
    )
    
    return LaunchDescription([
        port_arg,
        ros_sharp_communication_launch,
        node_unity_launcher,
    ])

