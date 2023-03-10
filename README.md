# ros_notes
ros notes and important keywords


## 1) Details for rospack ##
http://docs.ros.org/en/independent/api/rospkg/html/rospack.html
rospack is the ROS package management tool
1) ```rospack find``` : return the absolute path to a package
2) ```rospack depends``` : return a list of all of a package’s dependencies
3) ```rospack depends-on``` : return a list of packages that depend on the given package
4) ```rospack export``` : return flags necessary for building and linking against a package

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

### 15) Load newly created ROS node ###
```
rospack profile
```
### 16) Plot the topics ###
```
rosrun rqt_gui rqt_gui
```
