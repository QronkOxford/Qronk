#Really basic launch file for pybullet
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qronk_pybullet',
            namespace='qronk_pybullet',
            executable='startSim',
            name='sim')
    ])