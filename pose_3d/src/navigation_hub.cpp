#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>
#include <octomap/math/Utils.h>
#include <cmath>

#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <random>
#include <stdlib.h>
#include <time.h>

#include <boost/thread/thread.hpp>

#include <sensorCell.h>
#include <MinHeap.h>
#include <InfoNode.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <octomap_ros/conversions.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace octomap_msgs;

bool mapReceived;
Octomap mapMsg;
double res = 0.01;
double rFactor = 1 / res;
tf::Quaternion q;
move_base_msgs::MoveBaseGoal goal;
bool nbvReceived = false;
bool pinkReceived = false;
bool goalRunning = false;

void blob_cb(const geometry_msgs::Vector3ConstPtr& msg)
{
  pinkReceived = true;
}

void nbv_cb(const std_msgs::Float64MultiArrayConstPtr& nbv)
{
  ROS_INFO("Navigation Hub has received NBV");
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();  
  goal.target_pose.pose.position.x = double(nbv->data[0]);
  goal.target_pose.pose.position.y = double(nbv->data[1]);
  
  q.setRPY(0, 0, nbv->data[2]);
  goal.target_pose.pose.orientation.x = double(q.x());
  goal.target_pose.pose.orientation.y = double(q.y());
  goal.target_pose.pose.orientation.z = double(q.z());
  goal.target_pose.pose.orientation.w = double(q.w());
  
  nbvReceived = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_hub");
  
  ros::NodeHandle node;

  ros::Rate rate(10.0);
  
  MoveBaseClient ac("move_base", true);
  
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  tf::TransformListener listener;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::StampedTransform stransform;
  tf::Quaternion q;
  
  tf::Transform modOffset;
  tf::Transform modTransform;
  
  ros::Subscriber sub = node.subscribe("/nbv", 15, nbv_cb);
  ros::Subscriber blob_sub = node.subscribe("/blobPt", 15, blob_cb);

  ros::Publisher nbv_pub = node.advertise<std_msgs::Int32>("/nbv_iter", 15);
  
  bool pinkSeen = false;
  
  int wait;
  cin >> wait;
  
  while (ros::ok())
  {      
    std_msgs::Int32 temp;
    temp.data = 1;
    nbv_pub.publish<std_msgs::Int32>(temp);
    cout << "Publishhhhh" << endl;
    ros::spinOnce();
    
    ROS_INFO("Publishing NBV Request");
    ROS_INFO("Waiting for a target (to destroy)");
    
    while (!nbvReceived && ros::ok())
    {
      ros::spinOnce();
    }
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    nbvReceived = false;
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the base moved 1 meter forward");
      break;
    }
    else
    {
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
      
//       if (nbvReceived && !goalRunning)
//       {
// 	ROS_INFO("Sending goal");
// 	ac.sendGoal(goal);
// 	nbvReceived = false;
// 	goalRunning = true;
//       }
//       if (goalRunning)
//       {
// 	if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
// 	{
// 	  //cout << "Active" << endl;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
// 	{
// 	  //cout << "Pending" << endl;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
// 	{
// 	  cout << "Success" << endl;
// 	  goalRunning = false;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
// 	{
// 	  cout << "Aborted" << endl;
// 	  goalRunning = false;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
// 	{
// 	  cout << "Lost" << endl;
// 	  goalRunning = false;
// 	}
//       }
//       ros::spinOnce();
//     }
//     
//     cout << "Navigation Hub saw pink" << endl;
//     
//     pinkReceived = false;
//     
//     try{
//       ros::Time gTime = ros::Time::now();
//       listener.waitForTransform("map", "tag_0", gTime, ros::Duration(2.0));
//       listener.lookupTransform("map", "colorObj", gTime, stransform);
//     }
//     catch (tf::TransformException &ex) {
//       ROS_ERROR("%s",ex.what());
//       ros::Duration(1.0).sleep();
//       continue;
//     }
//     
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.header.stamp = ros::Time::now();  
//     goal.target_pose.pose.position.x = double(stransform.getOrigin().x());
//     goal.target_pose.pose.position.y = double(stransform.getOrigin().y());
//     
//     goal.target_pose.pose.orientation.x = 0;
//     goal.target_pose.pose.orientation.y = 0;
//     goal.target_pose.pose.orientation.z = 0;
//     goal.target_pose.pose.orientation.w = 1;
//     
//     ROS_INFO("Sending goal");
//     ac.sendGoal(goal);
//     
//     ac.waitForResult();
    
  }
  
  return 0;
}