The goal of ROS2 is to overcome the limitations of ROS, with particular attention to the creation of robotics products.
For instance, ROS2 includes real-time features and embedded security, and it is better prepared for the industrial world.

# ROS2 Basics #
## Basic Concepts ##
### 1) Launch executable ###
```ros2 run <package_name> <executable_file>```
### 2) Launch the launch file ###
```ros2 launch <package_name> <launch_file>```
### 3) Every python package consit of following files ###
####```package.xml``` - File containing meta-information about the package (maintainer of the package, dependencies, etc.).####
####```setup.py``` - File containing instructions for how to compile the package.####
####```setup.cfg``` - File that defines where the scripts will be installed.####
####```/<package_name>``` - This directory will always have the same name as your package. You will put all your Python scripts inside this folder. Note that it already contains an empty __init__.py file.####
