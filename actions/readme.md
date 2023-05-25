# ROS actions #

1) Actions are like asynchronous calls to services. An action is an asynchronous call to another node's functionality.  When your node calls an action, it doesn't necessarily have to wait for the action to complete.
2) cancel, feedback, goal, result and status: Are the messages used to comunicate with the Action Server. 
3)  The message of a topic is composed of a single part: the information the topic provides.
    The message of a service has two parts: the goal and the response.
    The message of an action server is divided into three parts: the goal, the result, and the feedback.
4) Action file should be in action directory within package directory
5) Example action file
```
#goal for the drone
int32 nseconds  # the number of seconds the drone will be taking pictures
---
#result
sensor_msgs/CompressedImage[] allPictures # an array containing all the pictures taken along the nseconds
---
#feedback
sensor_msgs/CompressedImage lastImage  # the last image taken

```
6) 
You can see in the previous step how the message is composed of three parts:

goal: Consists of a variable called nseconds of type Int32. This Int32 type is a standard ROS message, therefore, it can be found in the std_msgs package. Because it's a standard package of ROS, it's not needed to indicate the package where the Int32 can be found.
result: Consists of a variable called allPictures, an array of type CompressedImage[] found in the sensor_msgs package.
feedback: Consists of a variable called lastImage of type CompressedImage[] found in the sensor_msgs package.
You will learn in the second part of this chapter about how to create your own action messages. For now, you must only understand that every time you call an action, the message implied contains three parts, and that each part can contain more than one variable. 

7)
So the SimpleActionClient objects have two functions that can be used for knowing if the action that is being performed has finished, and how:

    wait_for_result(): This function is very simple. When called, it will wait until the action has finished and returns a true value. As you can see, it's useless if you want to perform other tasks in parallel because the program will stop there until the action is finished.
    get_state(): This function is much more interesting. When called, it returns an integer that indicates in which state is the action that the SimpleActionClient object is connected to:

0 ==> PENDING
1 ==> ACTIVE
2 ==> DONE
3 ==> WARN
4 ==> ERROR

This allows you to create a while loop that checks if the value returned by get_state() is 2 or higher. If it is not, it means that the action is still in progress, so you can keep doing other things.
