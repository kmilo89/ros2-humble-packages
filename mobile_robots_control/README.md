
# Mobile robots control

This ROS 2 package is used to simulate different environments and mobile robots in Gazebo simulator. Also, the servomotors can be simulated using ROS 2 Control.<br>

This repo structure is a combination of other common packages' structure into only one (robot_description, robot_models, robot_gazebo, robot_tools, robot_teleop, robot_control, etc).<br>

**NOTE:** this repo was tested on Ubuntu 22.04 LTS, with ROS 2 Humble and Gazebo simulator 11.


Author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado"<br>
Contact email: cmauricioae8@gmail.com<br>
Udemy profile: https://www.udemy.com/user/cruz-mauricio-arteaga-escamilla/<br>
LinkedIn: https://linkedin.com/in/cruz-mauricio-arteaga-escamilla/<br>

For more information, please refer to the following YouTube channel: https://www.youtube.com/channel/UCNmZp0rCuWxqaKVljny2zyg


# Information source
- URDF xacro:  
    + http://wiki.ros.org/urdf/XML
    + http://wiki.ros.org/urdf/Tutorials
- Gazebo list of materials: http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
- Gazebo color models: http://classic.gazebosim.org/tutorials?tut=color_model&cat=#AboutAmbient,Diffuse,Specular,andEmissive
- ROS Documentation: https://docs.ros.org/en/humble/index.html
- ROS 2 Control: https://control.ros.org/master/index.html,
https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html


# Installing ROS 2 packages

To avoid possible errors, please update your system and install the following ROS 2 dependencies.

```
sudo apt-get update
```

ROS 2 dependencies for robot description:

```bash
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf2-* ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-rviz-default-plugins
```

If the following error appears:<br>
_LookupError: Could not find the resource '<package_name>' of type 'packages'_

Try to install the correponding ROS dependency with

`sudo apt-get install ros-$ROS_DISTRO-<package-name>`

For example:

`sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui`

<br>


## Dependencies for ROS 2 control (Optional)

To clone the gazebo_ros2_control package from source, first, remove the 'gazebo_ros2_control' debian package, with:

`sudo apt remove ros-$ROS_DISTRO-gazebo-ros2-control`

Then, install it from source

```bash
cd colcon_ws/src
git clone -b $ROS_DISTRO https://github.com/ros-controls/gazebo_ros2_control
cd ..
colcon build --packages-select gazebo_ros2_control gazebo_ros2_control_demos --symlink-install
source install/setup.bash
```

To get an idea of what ROS control is, try some demos:
1. Diff drive

`ros2 launch gazebo_ros2_control_demos diff_drive.launch.py`

`ros2 run gazebo_ros2_control_demos example_diff_drive`

with 'diff_drive_controller.yaml'.

2. Trycycle drive

`ros2 launch gazebo_ros2_control_demos tricycle_drive.launch.py`

`ros2 run gazebo_ros2_control_demos example_tricycle_drive`

with 'tricycle_drive_controller.yaml'.

3. Velocity controller

`ros2 launch gazebo_ros2_control_demos cart_example_velocity.launch.py`

`ros2 run gazebo_ros2_control_demos example_velocity`

with 'cartpole_controller_velocity.yaml'.

4. Position controller (forward_command_controller)

`ros2 launch gazebo_ros2_control_demos gripper_mimic_joint_example.launch.py`

`ros2 run gazebo_ros2_control_demos example_gripper`

with 'gripper_controller.yaml'.

**Note:** there is another way to control a link, the effort controller sets the force and torque of the joint.

5. Trajectory controller (joint_trajectory_controller)

`ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py`

`ros2 run gazebo_ros2_control_demos example_position`

with 'cartpole_controller.yaml'.

This controller type uses a ROS action to move every joint. This way is used with MoveIt 2!!!



## Adding this repo

Please, paste this package in the src folder. Then:
```bash
cd ~/colcon_ws
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

If you already have all your dependencies, the console will return:<br>
#All required rosdeps installed successfully

**Note:** _This is made only once for the whole workspace._

Then, build colcon ws:
```bash
colcon build --packages-select mobile_robots_control --symlink-install
source install/setup.bash
```

**IMPORTANT:** This builds the package and sets a symbolic link to the python files (nodes and launch files). With this, re-build every time that a python file is modified, is not required.<br>
In ROS 2, launch files may be written in yaml, xml or python languages, but it is extremely recommended to use python. Also, the name of all launch files must finish with 'launch.py'. Otherwise, the file will not be recognized.

If some warnings appear, run `colcon build --packages-select mobile_robots_control --symlink-install` again and they will disappear.

**_Before starting to work with this package, it is required to modify the package.xml file with your user name as it is explained in the next section._**


## Meshes and models setup

To correctly visualize robots meshes and custom models in Gazebo with ROS 2, there are two options:

1. Add the 'GAZEBO_MODEL_PATH' environment variable into the _.bashrc_ file. This is, by pasting the followinf line
```
export GAZEBO_MODEL_PATH=~/colcon_ws/install/mobile_robots_control/share/
```
If that variable already exists, use ':' to add another path. For example:
`export GAZEBO_MODEL_PATH=~/.gazebo/models:<path_to_your_model>`

or

2. Add the 'gazebo_ros' tag into the 'export' tag of the _package.xml_ file. This is
```xml
<export>
  <build_type>ament_cmake</build_type>
  <gazebo_ros gazebo_model_path = "/home/<--user_name-->/colcon_ws/install/mobile_robots_control/share/" />
</export>
```

Re-build the colcon_ws
```
cd ~/colcon_ws
colcon build --packages-select mobile_robots_control --symlink-install
source install/setup.bash
```


# Launching the robot in Gazebo

The 'one_robot_gz_launch.py' launch file, opens Gazebo using an empty world, and spawns a robot which is selected in the file.

**_Note: The first time may take a while._**<br>

```
ros2 launch mobile_robots_control one_robot_gz_launch.py
```

To just visulize the robot and move its joints (if possible), run:

```
ros2 launch mobile_robots_control display_launch.py
```

To change the robot model, please edit the corresponding *launch.py file.


## Teleoperating the robot

To teleoperate the _differential_ mobile robot, please install 'rqt_robot_steering' debian package with:

`sudo apt install ros-humble-rqt-robot-steering*
`

and run it, with:

`rqt_robot_steering`

or

`rqt_robot_steering --force-discover`


Alternatively, to teleoperate both the differential and the omnidirectional mobile robot, use the package node:

`ros2 run mobile_robots_control omni_teleop_keyboard.py`

To publish a velocity from terminal:

`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}, angular: {z: 0.3}}"`

