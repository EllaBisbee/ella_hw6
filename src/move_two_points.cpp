/* made by: Ella Bisbee 6/5/18 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <ros/duration.h>
#include <vector>
#include <iostream>

//go_t0_point function takes x,y,z coordinates and actionclient and 
//sends goal to go to that point
void go_to_point(double x, double y, double z)
{
  //create action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> 
		ac("move_base", true);
  //create a goal                                                               
  move_base_msgs::MoveBaseGoal goal;  
  //set goal time stamp and frame id in the header, and x,y,z coords                              
  goal.target_pose.header.stamp = ros::Time::now();                             
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  //send goal and wait for result
  ros::Duration time_to_wait(60.0);
  ac.sendGoal(goal);
  ac.waitForResult(time_to_wait);
}


//main function
int main(int argc, char **argv)
{
  //set up ros 
  ros::init(argc, argv, "move_two_points");
  ros::NodeHandle thisNode;
 
  //say the action we're doing is to move the turlebot base,
  // and create an action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
			ac("move_base", true);  
  //create x,y,z coordinates for first and second position, with
  //hardcoded values I got from the ground floor map   
  double x1, x2, y1, y2, z1, z2;
  x1 = 1.6;
  y1 = -0.4;
  z1 = 0.0;
  x2 = -0.8;
  y2 = -0.8;
  z2 = 0.0; 
  //wait for server
  ac.waitForServer();  

  //while loop where robot will go back and fourth until end is signaled
  while(ros::ok())
  {
	go_to_point(x1, y1, z1);
	go_to_point(x2, y2, z2);
  }  

  return 0;
}