import os
import socket
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
import shutil
import xml.etree.ElementTree as ET

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')

        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if urdf_path == '':
            return
        urdf_file_name = os.path.basename(urdf_path)
        self.declare_parameter('package_name', '')
        package_name = self.get_parameter('package_name').get_parameter_value().string_value
        if package_name == '':
            return
        self.declare_parameter('unity_project_path', '')
        unity_project_path = self.get_parameter('unity_project_path').get_parameter_value().string_value
        if unity_project_path == '':
            return
        unity_project_path = os.path.expanduser(unity_project_path.rstrip('/'))
        self.get_logger().info(unity_project_path)
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
        if robot_fixed:
            robot_fixed_string = "true"
        else:
            robot_fixed_string = "false"

        self.get_logger().info("command start")
        
        # Copy URDF file
        os.makedirs(unity_project_path + "/Assets/Urdf/" + package_name, exist_ok=True)
        shutil.copy(urdf_path, unity_project_path + "/Assets/Urdf/" + package_name + "/" + urdf_file_name)

        # XMLファイルの読み込み
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # 全ての <mesh> タグの中の filename 属性の値をもとにstlファイルを転送
        filenames = [mesh.get('filename').replace('package://', '') for mesh in root.findall('.//mesh')]
        for filename in filenames:
            stl_package_name, stl_file_path = filename.split('/', 1)
            stl_directory_path, stl_file_name = os.path.split(filename)
            os.makedirs(unity_project_path + "/Assets/Urdf/" + package_name + "/" + stl_directory_path, exist_ok=True)
            package_path = os.path.join(get_package_share_directory(stl_package_name))
            shutil.copy(package_path.rstrip('/') + "/" + stl_file_path , unity_project_path + "/Assets/Urdf/" + package_name + "/" + filename)

        self.send_urdf_import_settings("URDF_IMPORT " + unity_project_path + "/Assets/Urdf/" + package_name + "/" + urdf_file_name + " " + str(robot_x) + " " + str(robot_y) + " " + str(robot_z) + " " + str(robot_roll) + " " + str(robot_pitch) + " " + str(robot_yaw) + " " + robot_fixed_string)
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

