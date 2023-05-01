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


