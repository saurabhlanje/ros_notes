# ros_notes
ros notes and important keywords


## 1) Details for rospack ##
http://docs.ros.org/en/independent/api/rospkg/html/rospack.html
rospack is the ROS package management tool
1) ```rospack find``` : return the absolute path to a package
2) ```rospack depends``` : return a list of all of a package’s dependencies
3) ```rospack depends-on``` : return a list of packages that depend on the given package
4) ```rospack export``` : return flags necessary for building and linking against a package
5) ```rospack profile```: Load newly created ROS node
6) ```rospack list```: list all available packages
7) ```rospack list | grep my_package```: List the packages which contains string "my_package"

## 2) View video feed captured by camera ##
```
rosrun rqt_image_view rqt_image_view
```
## 3) Verbose for launch file ##
For seeing more information with roslaunch you can try 
```roslaunch -v <package-name> <launch-name>``` to request verbosity output.

## 4) Graphical representation of nodes and topics##
```
rqt_graph
```
## 5) Predefined materials in gazebo to be used directly in URDF file 
```
vim /usr/share/gazebo-11/media/materials/scripts/gazebo.material
```

## 6) Tutorial Series ##

### 1) https://roboticscasual.com/robotics-tutorials/ ###

## 7)  Spwan model in gazebo ##
```
roslaunch gazebo_ros empty_world.launch
```
```
rosrun gazebo_ros spawn_model -file /<path-to-your-gazebo-urdf-file>/ur5_gazebo.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5
```
### 8) Spawn urdf file in launch file###
```
<param name="robot_description" command="cat '$(find robot_description)/urdf/robot.urdf'" />
```
### 9) Spawn xacro file in launch file ###
```
 <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find robot_description)/urdf/robot.xacro'"/>
 ```
 
 
 ### 10) Open xacro in rviz with launch file ###
 ```
<?xml version="1.0"?>
<launch>
    <arg name="rvizconfig" default="$(find robot_description)/rviz/urdf.rviz" />
    <param name="robot_description"
        command="$(find xacro)/xacro --inorder '$(find robot_description)/urdf/robot.xacro'"/>

  <!-- send fake joint values -->
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />

  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />

</launch>
```

### 11) Open urdf in rviz with launch file ###
```
<?xml version="1.0"?>
<launch>
    <arg name="rvizconfig" default="$(find robot_description)/rviz/urdf.rviz" />

  <param name="robot_description" command="cat '$(find robot_description)/urdf/robot.urdf'"/>

  <!-- send fake joint values -->
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />

  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />

</launch>
```
### 12) If gazebo simulation is unstable i.e. if robot behaves randomly not as per expection then increase the internation in the physics tab solver properties ###

### 13) Download gazebo models ###
```
http://machineawakening.blogspot.com/2015/05/how-to-download-all-gazebo-models.html
```

### 14) If you need multiple shapes in one link within urdf file it can be added with another visual tag with both tags named ###
explained here
```http://wiki.ros.org/urdf/XML/link```


### 15) Simplest ROS node in python ###
```#! /usr/bin/env python

import rospy

rospy.init_node("Saurabh")
print("Simplest node created")```

### 16) Plot the topics ###
```
rosrun rqt_gui rqt_gui
```

### 17) Check URDF file ###
```
check_urdf pr2.urdf
```
### 18) Check URDF file graphically ###
```
urdf_to_graphiz pr2.urdf
```
### 19) ROS control ###
#### 1) ros_control consist of controller interfaces, controller managers, transmissions, hardware_interfaces, and the control_toolbox ####
#### 2) os_control packages take the joint state data and an input set point(goal) as input, and sends the appropriate command to the actuators as an output (typically an effort command), in order to achieve the provided set point(goal) given the actual joint states. ####

### 20) Build only specific package ###
```catkin_make --only-pkg-with-deps package_name ```

### 21) Parameter server ###
```rosparam list```: Get list of all parameters
```rosparam get parameter_name``` : Get specific parameter value
```rosparam set parameter_name value : Set specific parameter value

### 22) Print only last value published by topic ###
```rostopic echo /topic_name -n1```

### 23) Get help on rostopic echo ###
``` rostopic echo -h ```
Similar way other commands in rostopic have help menu

### 24) Get details of various type of ros msgs ###
``` rosmsg show <message>```
Example
```rosmsg show std_msgs/Int32```
```rosmsg show geometry_msgs/``` press tab to know available msgs

### 25) ROS cheatsheet ###
```https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf```

### 26) xacro ###
####a) Convert xacro to urdf####
``` xacro --inorder model.xacro > model.urdf ```
####b) All formulas are computed using float####
####c) Link example with macros####
```
<xacro:property name="width" value="0.2" />
<xacro:property name="bodylen" value="0.6" />
<link name="base_link">
    <visual>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
    </collision>
</link>
```

####e) Example for link length ####
```
<xacro:property name=”robotname” value=”marvin” />
<link name=”${robotname}s_leg” />
```
This will generate
```<link name=”marvins_leg” />```

####f) Macro to generate the inertia property of link####
```
    <xacro:macro name="default_inertial" params="mass">
        <inertial>
                <mass value="${mass}" />
                <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                     iyy="1.0" iyz="0.0"
                     izz="1.0" />
        </inertial>
    </xacro:macro>
```
This can be used as
```
<xacro:default_inertial mass="10"/>
```

####b) ####

### 26) Get details of transforms ###
```
rosrun rqt_tf_tree rqt_tf_tree
```
### 27) Monitor all frames being broadcasted ###
```
rosrun tf tf_monitor
```
### 27) Publish static transform ###
```
rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms

```

### 28) Get graphical representation of URDF links and joints ###
```
urdf_to_graphiz dolly.urdf
evince dolly.urdf
```



