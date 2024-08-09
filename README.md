# ros2-humble-packages


## ROS2 for Beginners Level 2 - TF | URDF | RViz | Gazebo

sudo apt install ros-humble-urdf-tutorial

ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro

sudo apt install ros-humble-tf2-tools

ros2 launch urdf_tutorial display.launch.py model:=urdf/my_robot.urdf

## urdf Link documentation

https://wiki.ros.org/urdf/XML/link

## urdf Joint documentation

https://wiki.ros.org/urdf/XML/joint

### Example URDF file

```
<?xml version="1.0"?>
<robot name="my_robot">

    <material name="blue">
        <color rgba="0 0 0.5 1"/>
    </material>

    <material name="green">
        <color rgba="0 0.5 0 1"/>
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0"/> 
            <material name="blue"/>
        </visual>
    </link>

    <link name="second_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.2"/>
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0"/> 
            <material name="green"/>
        </visual>
    </link>


    <joint name="base_second_joint" type="revolute">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0 0 0.2" rpy="0 0 0"/> 
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" velocity="100" effort="100"/>
    </joint>

</robot>
```

### Example URDF file 2

```
<?xml version="1.0"?>
<robot name="my_robot">

    <material name="blue">
        <color rgba="0 0 0.5 1"/>
    </material>

    <material name="green">
        <color rgba="0 0.5 0 1"/>
    </material>

    <link name="base_footprint" />

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0"/> 
            <material name="blue"/>
        </visual>
    </link>

    <link name="right_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/> 
            <material name="green"/>
        </visual>
    </link>

    <link name="left_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/> 
            <material name="green"/>
        </visual>
    </link>

    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/> 
            <material name="green"/>
        </visual>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <origin xyz="-0.15 -0.225 0" rpy="1.57 0 0"/> 
        <axis xyz="0 0 1"/>
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <origin xyz="-0.15 0.225 0" rpy="1.57 0 0"/> 
        <axis xyz="0 0 1"/>
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel_link"/>
        <origin xyz="0.2 0 -0.05" rpy="0 0 0"/> 
    </joint>

</robot>
```

ros2 launch my_robot_description display.launch.xml 
ros2 launch my_robot_description display.launch.py

## Xacro

```
sudo apt install ros-humble-xacro
```

### Command to generate the urdf
```
ros2 param get /robot_state_publisher robot_description
```

## Create package

```
ros2 pkg create name_package # for pkg_description
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

## Clear ws
```
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
```



## Repositories Turtlebot3

https://github.com/ROBOTIS-GIT/turtlebot3
https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_description

## Adding Physical and Collision Properties to a URDF Model

https://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model

## List of moments of inertia

https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors



## Gazebo

1. ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)"
2. ros2 launch gazebo_ros gazebo.launch.py
3. ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot

## launch file
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

### Gazebo plugins

https://classic.gazebosim.org/tutorials?tut=ros_gzplugins

https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins


### gazebo camera frame is inconsistent with rviz + opencv convention

https://answers.ros.org/question/232534/gazebo-camera-frame-is-inconsistent-with-rviz-opencv-convention/


```
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header:{frame_id: arm_base_link}, joint_names: [arm_base_forearm_joint, forearm_hand_joint],points: [ {positions: {0.0, 0.0}} ]}' 
```

```
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header:{frame_id: base_footprint}, joint_names: [arm_base_forearm_joint, forearm_hand_joint],points: [ {positions: {0.0, 0.0}} ]}' 
```

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```











# Burger
```
# run burger
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

## Camera burger
```
sudo raspi-config
vcgencmd get_camera
# Output 
# supported=1 detected=1
sudo apt install ros-humble-image-tools
ros2 run image_tools cam2image

```

## Wii U Pro Controller 

```
sudo apt-get install ros-humble-joy
ros2 run joy joy_node
ros2 run joy game_controller_node
os2 launch turtlebot3_joy joy_teleop_launch.py
```


## SolidWorks to urdf
https://github.com/ros/solidworks_urdf_exporter/releases
https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html