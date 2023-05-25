# ROS actions #

1) Actions are like asynchronous calls to services. An action is an asynchronous call to another node's functionality.  When your node calls an action, it doesn't necessarily have to wait for the action to complete.
2) cancel, feedback, goal, result and status: Are the messages used to comunicate with the Action Server. 
3)  The message of a topic is composed of a single part: the information the topic provides.
    The message of a service has two parts: the goal and the response.
    The message of an action server is divided into three parts: the goal, the result, and the feedback.
