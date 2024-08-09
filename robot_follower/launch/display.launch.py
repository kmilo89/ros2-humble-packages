from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_path('robot_follower'), 'urdf', 'robot_follower.xacro')

    rviz_config_path = os.path.join(get_package_share_path('robot_follower'), 'rviz', 'urdf_config.rviz')

    robot_descripction = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable= 'robot_state_publisher',
        parameters= [{'robot_description': robot_descripction}]
    )

    joint_state_publisher_gui_node = Node(
        package = 'joint_state_publisher_gui',
        executable= 'joint_state_publisher_gui',
    )

    rviz2_node = Node(

        package = 'rviz2',
        executable= 'rviz2',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])