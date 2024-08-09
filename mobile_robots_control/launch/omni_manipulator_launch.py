"""
Launch file to spawn the mobile robot omni_manipulator in Gazebo. 
"""
__author__ = "C. Mauricio Arteaga-Escamilla from 'Robotica Posgrado' (Youtube channel)"

# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}, angular: {z: 0.3}}"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

package_name = 'mobile_robots_control'
robot_model = 'omni_manipulator'
robot_base_color = 'DarkGrey' #Gazebo color of the robot's main body
pose = ['0.0', '0.0', '0.0', '0.0'] #Initial robot pose in gz: x,y,z,th
gz_robot_name = robot_model #robot name used in Gazebo

#Launch argument
world_file = 'empty.world' #empty.world

        
def generate_launch_description():

    pkg_robot_simulation = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_robot_simulation, 'worlds', world_file), ''],
        description='Custom SDF world file')

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')


    robot_description_path =  os.path.join( pkg_robot_simulation, "urdf", robot_model + '.xacro', )
    
    robot_description = {"robot_description": xacro.process_file(robot_description_path, 
        mappings={'base_color' : robot_base_color}).toxml()}


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", gz_robot_name, 
            "-x", pose[0], "-y", pose[1], "-z", pose[2], "-Y", pose[3] ])
            
    #LOAD CONTROLLER NODES
    """load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )"""

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )
    
    load_pos_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'pos_controller'],
        output='screen'
    )
    
    omni_4_wheels = Node(
        package=package_name,
        executable='omni_4wheels',
        name='omni_4wheels',
        output='screen'
    )


    print("STARTING ALL NODES ...")

    return LaunchDescription([
        world_arg,
        simu_time,
        gazebo_node,
        robot_state_publisher_node,
        robot_spawner,
        #load_joint_state_controller, #Ommited since Gazebo plugin is used (libgazebo_ros_joint_state_publisher)
        load_velocity_controller,
        load_pos_controller,
        omni_4_wheels
    ])
    

