from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    algo = LaunchConfiguration('algorithm', default='antipodal')

    nodes = [
        Node(package='grasp_perception', executable='obstacle_node', output='screen'),
        Node(package='grasp_algorithms', executable='antipodal_node', output='screen', parameters=[{'algorithm': algo}]),
        Node(package='grasp_executor', executable='panda_grasp_executor', output='screen'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('algorithm', default_value='antipodal'),
        *nodes
    ])
