from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_marker_tracking')

    world = os.path.join(pkg_share, 'worlds', 'calibration_5x5_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'demo.rviz')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/overhead_camera/image'],
            output='screen'
        ),

        Node(
            package='robot_marker_tracking',
            executable='marker_tracker_node',
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])