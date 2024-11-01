import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'unity_ros2_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('unity_scripts/*.py')),
        (os.path.join('share',package_name), glob('unity_scripts/*.sh')),
        (os.path.join('share',package_name, 'meshes', 'USD'), glob('meshes/USD/*.usd')),
        (os.path.join('share',package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launcher = unity_ros2_scripts.launcher:main',
            'spawn_robot = unity_ros2_scripts.spawn_robot:main',
            'prepare_sensors = unity_ros2_scripts.prepare_sensors:main',
            'prepare_robot_controller = unity_ros2_scripts.prepare_robot_controller:main',
            'add_usd = unity_ros2_scripts.add_usd:main',
        ],
    },
)