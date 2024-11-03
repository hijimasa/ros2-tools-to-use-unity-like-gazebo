import os
import socket
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')

        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        if robot_name == '':
            return
        self.declare_parameter('package_name', '')
        package_name = self.get_parameter('package_name').get_parameter_value().string_value
        if package_name == '':
            return
        self.declare_parameter('node_name', 'robot_state_publisher')
        node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self.declare_parameter('param_name', 'robot_description')
        param_name = self.get_parameter('param_name').get_parameter_value().string_value
        self.declare_parameter('asset_path', str(os.path.expanduser("~/work/Robot_Unity_App/Assets/Urdf")))
        asset_path = self.get_parameter('asset_path').get_parameter_value().string_value
        self.declare_parameter('x', 0.0)
        robot_x = self.get_parameter('x').get_parameter_value().double_value
        self.declare_parameter('y', 0.0)
        robot_y = self.get_parameter('y').get_parameter_value().double_value
        self.declare_parameter('z', 0.0)
        robot_z = self.get_parameter('z').get_parameter_value().double_value
        self.declare_parameter('R', 0.0)
        robot_roll = self.get_parameter('R').get_parameter_value().double_value
        self.declare_parameter('P', 0.0)
        robot_pitch = self.get_parameter('P').get_parameter_value().double_value
        self.declare_parameter('Y', 0.0)
        robot_yaw = self.get_parameter('Y').get_parameter_value().double_value
        self.declare_parameter('fixed', False)
        robot_fixed = self.get_parameter('fixed').get_parameter_value().bool_value

        self.get_logger().info("command start")
        self.send_urdf_import_settings("URDF_IMPORT " + robot_name + ":" + package_name + " " + node_name + ":" + param_name + " " + asset_path + " " + str(robot_x) + " " + str(robot_y) + " " + str(robot_z) + " " + str(robot_roll) + " " + str(robot_pitch) + " " + str(robot_yaw))
        self.get_logger().info("command end")

    def __del__(self):
        pass

    def send_urdf_import_settings(self, urdf_import_settings):
        host = 'localhost'  # Unity Editorを実行しているPCのIPアドレス
        port = 5000

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((host, port))
            s.sendall(urdf_import_settings.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()
    minimal_publisher.get_logger().info("node start")

    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

