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
For seeing more information with roslaunch you can try 
```roslaunch -v <package-name> <launch-name>``` to request verbosity output.
