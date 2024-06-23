# 1) Open only gazebo
Gazebo can be independently started by following command in terminal #
```
gazebo
```

# 2) Open some world in gazebo #
```
gazebo worlds/pioneer2dx.world
```
```
gzserver <world_filename>
```
Gazebo can be started with loading a world with in the simulator
Argument provided to the gazebo command is the path of the world to be opened
All worlds are store at below location in ros noetic
```
/usr/share/gazebo-11/worlds
```
# 3) Start Gazebo server #
```
gzserver
```
This executable runs the physics update-loop and sensor data generation.

# 4) Start gazebo clidet #
This is just required for the user inferface only
```
gzclient
```
Gazebo simulator can run in headless mode as well

# 5)  Stop gzclient #
```
killall -9 gzclient
```
# 6) Gazebo Components #
This section describes each of the items involved in running a Gazebo simulation.
## 6.1 World files ##
* The world description file contains all the elements in a simulation, including robots, lights, sensors, and static objects.  
* This file is formatted using SDF (Simulation Description Format), and typically has a .world extension.
* The Gazebo server (gzserver) reads this file to generate and populate a world.
## 6.2 Model files ##
* A model file uses the same SDF format as world files, but should only contain a single <model> ... </model>
* The purpose of these files is to facilitate model reuse, and simplify world files. Once a model file is created, it can be included in a world file using the following SDF syntax:
  ```
  <include>
  <uri>model://model_file_name</uri>
  </include>
  ```
* A number of models are provided in the online model database (in previous versions, some example models were shipped with Gazebo). Assuming that you have an Internet connection when running Gazebo, you can insert any model from the database and the necessary content will be downloaded at runtime.
## 6.3 Gazebo Environment Variables ##
* GAZEBO_MODEL_PATH: colon-separated set of directories where Gazebo will search for models
* GAZEBO_RESOURCE_PATH: colon-separated set of directories where Gazebo will search for other resources such as world and media files.
* GAZEBO_MASTER_URI: URI of the Gazebo master. This specifies the IP and port where the server will be started and tells the clients where to connect to.
* GAZEBO_PLUGIN_PATH: colon-separated set of directories where Gazebo will search for the plugin shared libraries at runtime.
* GAZEBO_MODEL_DATABASE_URI: URI of the online model database where Gazebo will download models from.
## 6.4 Gazebo Plugins ##
# Gazebo screen shots are saved at following location #
```
~/.gazebo/pictures
```
Path for saving video is asked when video recoding is stopped
# 7) ROS gazebo #
## 7.1) Start Gazebo from ROS ##
```
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true verbose:=true gui_required:=true
```
* paused = Start Gazebo in a paused state (default false)
* use_sim_time = Tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock (default true)
* gui = Launch the user interface window of Gazebo (default true)
* debug = Start gzserver (Gazebo Server) in debug mode using gdb (default false)
* verbose = Run gzserver and gzclient with --verbose, printing errors and warnings to the terminal (default false)
* server_required = Terminate launch script when gzserver (Gazebo Server) exits (default false)
* gui_required = Terminate launch script when gzclient (user interface window) exits (default false)

## 7.2) Using roslaunc file ##
```
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/mud.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="recording" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```
Arguments and their effect is same as mentioned above.

  

