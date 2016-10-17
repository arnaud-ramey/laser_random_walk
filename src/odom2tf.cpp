/* http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
   http://www.arscontrol.org/uploads/alan/feedback_linearization.cpp
*/
//Include some standard ROS library
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//Global variables
tf::TransformBroadcaster* tf_pub;
geometry_msgs::TransformStamped odom_trans;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) //It is a function who reads the value of odometry message
{
  //  ROS_INFO("Odometry message received : '%s'->'%s'!",
  //           msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
  odom_trans.header = msg->header;
  odom_trans.header.stamp = ros::Time::now();   // refresh stamp
  odom_trans.header.seq++;
  odom_trans.child_frame_id = msg->child_frame_id;
  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;
  odom_trans.transform.rotation = msg->pose.pose.orientation;
  tf_pub->sendTransform(odom_trans);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "odom2tf"); //Initialise and create a ROS node
  tf_pub = new tf::TransformBroadcaster();
  ros::NodeHandle node; //Active the node and allocate the memory
  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("/odom", 100, &poseCallback); //Associates the subscriber of the object node with the subscriber that we have create. It will receive odometry message with 100 elements queue.
  ros::spin();
  delete tf_pub;
}
