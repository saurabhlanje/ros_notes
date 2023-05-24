#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <services_quiz/BB8CustomServiceMessage.h>
#include <unistd.h>

//the publisher is defined over here so that it is accessible in all functions

ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;

using namespace std;

bool my_callback(services_quiz::BB8CustomServiceMessage::Request  &req,
                 services_quiz::BB8CustomServiceMessage::Response &res)
{  
  ROS_INFO("The Service move_bb8_in_square_custom has been called");
  float radius = req.side;
  int reps = req.repetitions;
  for (int i=0; i < reps; ++i)
  {
    ROS_INFO("Moving Forward...");
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
    usleep(radius*1000000); // We multiply for 1000000 because the time is set in microseconds
    ROS_INFO("Rotating...");
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0.2;
    vel_pub.publish(vel_msg);
    usleep(4000000);
    ROS_INFO("Moving Forward...");
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
    usleep(radius*1000000);
    ROS_INFO("Rotating...");
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0.2;
    vel_pub.publish(vel_msg);
    usleep(4000000);
    ROS_INFO("Moving Forward...");
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
    usleep(radius*1000000);
    ROS_INFO("Rotating...");
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0.2;
    vel_pub.publish(vel_msg);
    usleep(4000000);
    ROS_INFO("Moving Forward...");
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
    usleep(radius*1000000);
    ROS_INFO("Rotating...");
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0.2;
    vel_pub.publish(vel_msg);
    usleep(4000000);
    ROS_INFO("Stopping...");
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
    usleep(2000000);
  }

  res.success = true;
  ROS_INFO("Finished service move_bb8_in_square_custom");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_move_bb8_in_square_custom_server");
  ros::NodeHandle nh;

  ros::ServiceServer my_service = nh.advertiseService("/move_bb8_in_square_custom", my_callback);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ROS_INFO("Service /move_bb8_in_square_custom Ready");
  ros::spin();

  return 0;
}
