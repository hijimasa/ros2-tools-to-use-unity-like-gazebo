import os
import glob
import signal
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
from os.path import expanduser

def find_unity_executable(base_dir="~/Unity/Hub/Editor"):
    # Expand user directory in the base path
    base_dir = os.path.expanduser(base_dir)
    
    # Search for the Unity executable within any version folder
    unity_executable_pattern = os.path.join(base_dir, "*/Editor/Unity")
    unity_executables = glob.glob(unity_executable_pattern)
    
    if unity_executables:
        print("Unity executable(s) found:")
        for executable in unity_executables:
            print(executable)
        return str(unity_executables[0])
    else:
        print("No Unity executable found.")
        return None

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')
        self.proc = None

        self.declare_parameter('unity_path', '')
        unity_path = self.get_parameter('unity_path').get_parameter_value().string_value
        found_unity_path = find_unity_executable()
        if (unity_path == '') and (not found_unity_path == None):
            unity_path = found_unity_path
        if not os.path.isfile(unity_path):
            self.get_logger().fatal('Unity not found!!')
            return

        self.declare_parameter('project_path', str(os.path.expanduser("~")) + '/work/Robot_Unity_App/')
        project_path = self.get_parameter('project_path').get_parameter_value().string_value

        self.declare_parameter('open_file_path', '')
        open_file_path = self.get_parameter('open_file_path').get_parameter_value().string_value
                
        command = [unity_path, "-projectPath", project_path]
        if not open_file_path == '':
            command.append('-openfile')
            command.append(open_file_path)
        print(command)
        self.proc = subprocess.Popen(command, preexec_fn=os.setsid)        
    
    def __del__(self):
        if not self.proc == None:
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

