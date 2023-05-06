The goal of ROS2 is to overcome the limitations of ROS, with particular attention to the creation of robotics products.
For instance, ROS2 includes real-time features and embedded security, and it is better prepared for the industrial world.

# ROS2 Basics #
## Basic Concepts ##
### 1) Launch executable ###
```ros2 run <package_name> <executable_file>```
### 2) Launch the launch file ###
```ros2 launch <package_name> <launch_file>```
### 3) Every python package consit of following files ###

#### ```package.xml``` - File containing meta-information about the package (maintainer of the package, dependencies, etc.). ####
#### ```setup.py``` - File containing instructions for how to compile the package. ####
#### ```setup.cfg``` - File that defines where the scripts will be installed. ####
#### ```/<package_name>``` - This directory will always have the same name as your package. You will put all your Python scripts inside this folder. Note that it already contains an empty __init__.py file. ####
#### Some time additional folder like launch folder might be also present which consist of launch file ####


### 4) Create ros2 package ###
```ros2 pkg create --build-type ament_python <package_name> --dependencies <package_dependencies>```
Package name and package dependencies is self explanatory
build type has two types
#### 1) amet_cmake ####
This for C, C++ cmake
#### 2) amet_python ###
This is for python

### 5) Build package ###
1) Go to workspace directory
2) ``` colcon build ```
3) After building all packages in workspace source the setup.bash file from install folder


### 6) Source bash file of workspace ###
```source install/setup.bash```

### 7) List all packages ###
```ros2 pkg list```
### 8) Find package with specific keywords ###
```ros2 pkg list | grep my_package```

### 9) Build only some specific package and their dependencies ###
```colcon build --packages-select <package_name>```

### 10) Example launch file ###
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            output='screen'),
    ])
```
### 11) List all Nodes ###
``` ros2 node list ```

### 12)  List all packages with their executables ###
```ros2 pkg executables```

### 13) No roscore in ROS2 ###
roscore was a single point failure in ros1. It has been removed in ROS2

### 14) ROS tutorials ###
```https://medium.com/@nullbyte.in```
```https://docs.ros.org/en/humble/Tutorials.html```

### 15) List commands ###
```ros2 node list```

```ros2 topic list```

For listing topic type with topic

```ros2 topic list -t```

```ros2 service list```

```ros2 action list```

### 16) Launch rqt ###
```rqt```
rqt provides interface for all ros related things like topics, services, actions, plots

### 17) Remap topic to some other name ###
```ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel```

### 18) Get topic info ###
```ros2 node info <node_name>```

### 19) Graph for nodes and topics ###
```rqt_graph```

### 20) observe topic on cli ###
```ros2 topic echo <topic_name>```

### 21)  Get topic info ###
```ros2 topic info /turtle1/cmd_vel```

### 22) Get the pattern of msg ###
```ros2 interface show geometry_msgs/msg/Twist```

Get prototype of the message
```ros2 interface proto geometry_msgs/msg/Twist```

### 22) Publish topic through cli ###
```ros2 topic pub <topic_name> <msg_type> '<args>'```

### 23) Publish topic at 1 hz ###
```ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"```

### 24) Publish topic for once ###
```ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"```

### 25) Differences between ROS 1 and ROS 2
#### 1) No roscore in ros 2 ####
#### 2) All commands starts with ros2 and there is space between ros2 and the next part of command ####
#### 3) ros client libraries are implemented in C only, for python client library only wrapper is made over C. This is just to make it simple and easy to implement any changes to the C code. The functionality remains same irrespective of Node being a python node or C++ node####

### Get help for topic ###
``` ros2 topic -h```

### To publish topic through cli with auto completion you have to type up to this ###
```ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{ ``` and then press tab to get details of interface. It will autocomplete and you will get something like this 
``` ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"```

### Format for topic publishing ###
``` ros2 topic pub <topic_name> <interface_name> <message>```

