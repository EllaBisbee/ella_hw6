/*made by: Ella Bisbee 6/7/18 */                                                
                                                                                
#include <ros/ros.h>                                                            
#include <move_base_msgs/MoveBaseAction.h>                                               
#include <people_msgs/PositionMeasurement.h>                                    
#include <people_msgs/PositionMeasurementArray.h>                               
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>                                                              
#include "geometry_msgs/Pose.h"                                                 
#include "geometry_msgs/PoseArray.h"           
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>                                                                
#include <math.h>
#include <vector>

using namespace std;

//global variables for the turtlebots current position,
//so they cam be accessed in the subscriber function
double curr_x;
double curr_y;
double curr_z;

void followThatLeg(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
  if (!(msg->people.empty()))
{
  //setup the action and goal to move the robot
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
		ac("move_base", true);
  ac.waitForServer();
  move_base_msgs::MoveBaseGoal goal;
  //extract relevant data from msg
  double person_x = msg->people[0].pos.x;
  double person_y = msg->people[0].pos.y;
  double person_z = msg->people[0].pos.z;

  //send goal to move close to the person based on curr position, 
  //give in a goal along the line between the two points that is .5 away 
  //from the person, using triginometry. If robot is very close, it sets
  //the goal to be the halfway point betwen the robot and person
  float distance = sqrt(pow(person_x, 2) + pow(person_y, 2));
  if (distance >= .1)
  {
	goal.target_pose.header.frame_id = "/base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = (1 - .05/distance) * person_x;
	goal.target_pose.pose.position.y = (1 - .05/distance) * person_y;
	goal.target_pose.pose.position.z = person_z;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	ac.sendGoal(goal);
	ac.waitForResult();
  }  
 }
}

void getCurrPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //get curr x, y, z position from msg
  curr_x = msg->pose.pose.position.x;
  curr_y = msg->pose.pose.position.y;
  curr_z = msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{
  //initializing global variables
  curr_x = 0;
  curr_y = 0;
  curr_z = 0;
  ros::init(argc, argv, "leg_follower");
  ros::NodeHandle node;
  ros::Subscriber leg_sub = node.subscribe("people_tracker_measurements",
					 1000, followThatLeg);
  ros::Subscriber amcl_sub = node.subscribe("amcl_pose", 1000, getCurrPos);
  ros::spin(); 
  return 0;
}  
