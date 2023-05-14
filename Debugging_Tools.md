ROS Debugging Messages
In ROS logs case, there are five levels. Each level includes deeper levels.
So, for example, if you use Error level, all the messages for Error and Fatal will be shown.
If your level is Warning, then all the messages for levels Warning, Error and Fatal will be shown.

Example node for log levels
```
#! /usr/bin/env python

import rospy
import random
import time

# Options: DEBUG, INFO, WARN, ERROR, FATAL
rospy.init_node('log_demo', log_level=rospy.DEBUG)
rate = rospy.Rate(0.5)

#rospy.loginfo_throttle(120, "DeathStars Minute info: "+str(time.time()))

while not rospy.is_shutdown():
    rospy.logdebug("There is a missing droid")
    rospy.loginfo("The Emperors Capuchino is done")
    rospy.logwarn("The Revels are coming time "+str(time.time()))
    exhaust_number = random.randint(1,100)
    port_number = random.randint(1,100)
    rospy.logerr(" The thermal exhaust port %s, right below the main port %s", exhaust_number, port_number)
    rospy.logfatal("The DeathStar Is EXPLODING")
    rate.sleep()
    rospy.logfatal("END")


````

rosout node is the console of all the logging mechanisms in ros

RQT console
rqt_console is a viewer in the rqt package that displays messages being published to rosout(which is cosole for all log messages).
It collects messages over time, and lets you view them in more detail, as well as allowing you to filter messages by various means. 
It is a gui for viewing log messages
The rqt_console window is divided into three subpanels.

    The first panel outputs the logs. It has data about the message, severity/level, the node generating that message, and other data. Is here where you will extract all your logs data.

    The second one allows you to filter the messages issued on the first panel, excluding them based on criteria such as: node, severity level, or that it contains a certain word. To add a filter, just press the plus sign and select the desired one.

    The third panel allows you to highlight certain messages, while showing the other ones.

rqt_plot is used to plot all the data from topics
rqt_graph tells about the interconnection of node in terms of topics

Record Experimental Data and Rosbags

1. **Record**: Start recording messages from specified topics to a bag file.
```
rosbag record -O <output_file> <topic1> <topic2> ...
```
Replace `<output_file>` with the desired name of the bag file to be generated. `<topic1>`, `<topic2>`, etc. represent the topics you want to record.

2. **Play**: Play back the messages stored in a bag file.
```
rosbag play <bag_file>
```
Replace `<bag_file>` with the name of the bag file you want to play.

3. **Info**: Display information about a bag file.
```
rosbag info <bag_file>
```
Replace `<bag_file>` with the name of the bag file you want to get information about.

4. **Filter**: Apply a filter to a bag file and generate a new filtered bag file.
```
rosbag filter <input_bag> <output_bag> <filter_expression>
```
Replace `<input_bag>` with the name of the input bag file, `<output_bag>` with the name of the output filtered bag file, and `<filter_expression>` with the filter expression to be applied.

5. **Fix**: Fix a bag file by rewriting it with a consistent index.
```
rosbag fix <input_bag> <output_bag>
```
Replace `<input_bag>` with the name of the input bag file and `<output_bag>` with the name of the output fixed bag file.

6. **Compress**: Compress a bag file using bz2 compression.
```
rosbag compress <bag_file>
```
Replace `<bag_file>` with the name of the bag file you want to compress.

7. **Decompress**: Decompress a bag file.
```
rosbag decompress <bag_file>
```
Replace `<bag_file>` with the name of the compressed bag file you want to decompress.

These commands should help you get started with using `rosbag` for recording, playing, filtering, and managing bag files in ROS.
8. Record all topics
```
rosbag record -a
```



