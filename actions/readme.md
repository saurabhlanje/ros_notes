# ROS actions #

1) Actions are like asynchronous calls to services. An action is an asynchronous call to another node's functionality.  When your node calls an action, it doesn't necessarily have to wait for the action to complete.
2) cancel, feedback, goal, result and status: Are the messages used to comunicate with the Action Server. 
3)  The message of a topic is composed of a single part: the information the topic provides.
    The message of a service has two parts: the goal and the response.
    The message of an action server is divided into three parts: the goal, the result, and the feedback.
4) Action file should be in action directory within package directory
5) Example action file
```
user ~ $ roscd ardrone_as/action; cat Ardrone.action
#goal for the drone
int32 nseconds  # the number of seconds the drone will be taking pictures
---
#result
sensor_msgs/CompressedImage[] allPictures # an array containing all the pictures taken along the nseconds
---
#feedback
sensor_msgs/CompressedImage lastImage  # the last image taken

```
