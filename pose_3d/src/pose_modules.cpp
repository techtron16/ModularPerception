// pose_3d.cpp
//

//#include "stdafx.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Pose6D.h>
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
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <../../opt/ros/indigo/include/tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <octomap_ros/conversions.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace octomap_msgs;

vector<int> logAdd;
vector<vector<int> > visMap;

const double res = 0.02;
const double rFactor = 1 / res;
const int searchDepth = 16; //finest search depth is 16
const double sRes = res * pow(2, (16 - searchDepth));
const int sFactor = round(1 / sRes);
const int nRayLength = 75;
const double range = nRayLength * sRes;

const int gridDepth = 14; //finest search depth is 16
const double gRes = res * pow(2, (16 - gridDepth));

int pow1 = pow(2 * nRayLength + 1, 2);
int pow0 = (2 * nRayLength + 1);

//vector<vector<vector<vector<float>>>> infoChunk;
vector<vector<float> > infoChunk;
vector<int> newMap;

vector<int> optiMap;
vector<vector<int> > optiRays;

vector<SensorCell> sensorModel;
vector<vector<float>> sensorChunk(pow(2 * nRayLength + 1, 3), { 0, 0 });

float poseMap[180][69];

double camHeight = 0.115;

point3d ibxMin;
point3d ibxMax;
int boxX, boxY, boxZ;
int prodZ;
int maxIdx;
int deltaIdx;

const double lFilled = 1.39;
const double lEmpty = 1.39;

ros::Publisher cmd_pub;

bool mapReceived = false;
bool startAlg = false;

//OcTree tree(res);
Octomap mapMsg;

nav_msgs::OccupancyGrid map2d;

void map_cb(const octomap_msgs::OctomapConstPtr& map)
{
  cout << ".";
  mapMsg = *map;
  mapReceived = true;
  
  //OcTree treeTemp(*binaryMsgToMap(*map));
    
  //treeTemp.writeBinary("testMap.bt");
}

double getInfoGain(OcTree *tree, OcTree *tree2, point3d nextPos, float *infoYaw, float *infoPitch)
{//Use Vectorization Method to Compute Info Gain for 3D location
  double totStuff = 0;
  
  //--------------
  //New Batch Vectorization Method using dynamically sized infoChunk array
  //--------------
  
  point3d newNext = tree->keyToCoord(tree->coordToKey(nextPos), searchDepth);
  
  int deltaMinX = round(max(0.0, min((double)ibxMax.x() - ibxMin.x(), newNext.x() - range - ibxMin.x())) * sFactor);
  int deltaMinY = round(max(0.0, min((double)ibxMax.y() - ibxMin.y(), newNext.y() - range - ibxMin.y())) * sFactor);
  int deltaMinZ = round(max(0.0, min((double)ibxMax.z() - ibxMin.z(), newNext.z() - range - ibxMin.z())) * sFactor);
  
  int deltaMaxX = round(max(0.0, min((double)ibxMax.x() - ibxMin.x(), newNext.x() + range - ibxMin.x())) * sFactor);
  int deltaMaxY = round(max(0.0, min((double)ibxMax.y() - ibxMin.y(), newNext.y() + range - ibxMin.y())) * sFactor);
  int deltaMaxZ = round(max(0.0, min((double)ibxMax.z() - ibxMin.z(), newNext.z() + range - ibxMin.z())) * sFactor);
  
  point3d delta = (newNext - point3d(1, 1, 1) * range - ibxMin) * sFactor;
  int highBound = 0;
  int highY = 0;
  int highX = 0;
  int lowBound = pow(2 * nRayLength + 1, 3);
  int lowY = 2 * nRayLength + 1;
  int lowX = 2 * nRayLength + 1;
  
  int x = deltaMinX - round(delta.x());
  int y = deltaMinY - round(delta.y());
  int z = deltaMinZ - round(delta.z());
  if (x >= 0 && x < 2 * nRayLength + 1 && y >= 0 && y < 2 * nRayLength + 1 && z >= 0 && z < 2 * nRayLength + 1)
  {
    lowBound = x + y * pow0 + z * pow1;
    lowY = y;
    lowX = x;
  }
  
  x = deltaMaxX - round(delta.x());
  y = deltaMaxY - round(delta.y());
  z = deltaMaxZ - round(delta.z());
  if (x >= 0 && x < 2 * nRayLength + 1 && y >= 0 && y < 2 * nRayLength + 1 && z >= 0 && z < 2 * nRayLength + 1)
  {
    highBound = x + y * pow0 + z * pow1;
    highY = y;
    highX = x;
  }
  else
  {
    //cout << "AHHHHHH!!!" << endl;
    //cout << deltaMaxX << ", " << deltaMaxY << ", " << deltaMaxZ << endl;
  }
  
  for (int i = 0; i < 180; i++)
  {
    for (int j = 0; j < 69; j++)
    {
      poseMap[i][j] = 0;
    }
  }
  
  vector<int>::iterator idxIter;
  vector<vector<int>>::iterator visIter;
  
  //cout << "New Next: " << newNext.x() << ", " << newNext.y() << ", " << newNext.z() << endl;
  
  prodZ = boxX * boxY;
  maxIdx = boxX * boxY * boxZ;
  //deltaIdx = round((nextPos.x() - ibxMin.x()) / sRes) - nRayLength + (round((nextPos.y() - ibxMin.y()) / sRes) - nRayLength) * boxX + (round((nextPos.z() - ibxMin.z()) / sRes) - nRayLength) * prodZ;
  int deltaX = round((ibxMin.x() - newNext.x()) * sFactor) + nRayLength;
  int deltaY = round((ibxMin.y() - newNext.y()) * sFactor) + nRayLength;
  int deltaZ = round((ibxMin.z() - newNext.z()) * sFactor) + nRayLength;
  
  //cout << "Deltas: " << deltaX << ", " << deltaY << ", " << deltaZ << endl;
  
  for (idxIter = optiMap.begin(), visIter = optiRays.begin(); idxIter != optiMap.end(); idxIter++, visIter++)
  {
    if (*idxIter < lowBound) continue;
    else if (*idxIter >= highBound) break;
    
    int y = (*idxIter % pow1) / pow0;
    if (y < lowY || y >= highY) continue;
    
    int x = (*idxIter % pow1) % pow0;
    if (x < lowX || x >= highX) continue;
    
    int z = *idxIter / pow1;
    
    x -= deltaX;
    y -= deltaY;
    z -= deltaZ;
    
    int infoIdx = z * prodZ + y * boxX + x;
    
    double cellIG = infoChunk[infoIdx][0];
    
    if (cellIG > 0.012)
    {
      int z1 = *idxIter / pow1;
      int y1 = ((*idxIter % pow1) / pow0);
      int x1 = (*idxIter % pow1) % pow0;
      
      int yaw = round(RAD2DEG(atan2(y1 - nRayLength, x1 - nRayLength)) / 2) * 2;
      int pitch = round(RAD2DEG(atan2(double(z1 - nRayLength), double(sqrt(pow(x1 - nRayLength, 2) + pow(y1 - nRayLength, 2))))) / 2) * 2;
      
      for (vector<int>::iterator rayIter = visIter->begin(); rayIter != visIter->end(); rayIter++)
      {//Loop through ray cells to calculate pVis for factoring into cellIG
	if (*rayIter < lowBound) continue;
	else if (*rayIter >= highBound) break;
	
	int y = (*rayIter % pow1) / pow0;
	if (y < lowY || y >= highY) continue;
	
	int x = (*rayIter % pow1) % pow0;
	if (x < lowX || x >= highX) continue;
	
	int z = *rayIter / pow1;
	
	x -= deltaX;
	y -= deltaY;
	z -= deltaZ;
	
	int infoIdx = z * prodZ + y * boxX + x;
	
	cellIG *= (1 - infoChunk[infoIdx][1]);
	if (cellIG < 0.01)
	{
	  cellIG = 0;
	  break;
	}
	
      }
      
      totStuff += cellIG;
      //objStuff += pObj * H0;
      //objCount++;
      
      for (int thIdx = yaw - 28; thIdx <= yaw + 28; thIdx += 2)
      {
	for (int phIdx = max(pitch - 22, -68); phIdx <= min(pitch + 22, 68); phIdx += 2)
	{
	  int theta = thIdx;
	  int phi = phIdx + 68;
	  
	  if (theta < 0)  theta += 360;
	  else if (theta > 360) theta -= 360;
	  
	  //cout << "Angles: " << thIdx << ", " << phIdx << endl;
	  //cout << "Indices: " << t << ", " << p << endl;
	  
	  poseMap[theta / 2][phi / 2] += cellIG;
	}
      }
    }
  }
  
  //cout << "totStuff: " << totStuff << endl;
  //cout << "Test New Time: " << double(clock() - bbxStart) / CLOCKS_PER_SEC << endl;
  
  double posedIG = 0;
  for (int yaw = 0; yaw < 360; yaw += 2)
  {
    for (int pitch = -68; pitch <= 68; pitch += 2)
    {
      if (poseMap[yaw / 2][(pitch + 68) / 2] > posedIG)
      {
	posedIG = poseMap[yaw / 2][(pitch + 68) / 2];
	*infoYaw = yaw;
	*infoPitch = pitch;
      }
    }
  }
  
  //cout << "Time 0 : " << (double)(t0 - bbxStart) / CLOCKS_PER_SEC << endl;
  //cout << "Time 1 : " << (double)(t1 - t0) / CLOCKS_PER_SEC << endl;
  //cout << "Time 2 : " << (double)(t2 - t1) / CLOCKS_PER_SEC << endl;
  //cout << "Time 2 : " << (double)(t3 - t2) / CLOCKS_PER_SEC << endl;
  
  //cout << posedIG << endl;
  //return posedIG;
  
  //cout << "Optimized: " << totStuff << " and " << objStuff << " from " << objCount << " and " << totCount << endl;
  
  return totStuff;
}

int sign(double x)
{
  return -(x < 0) + (x > 0);
}

void moveBase(point3d goalPt, double goalYaw, tf::TransformListener *listener)
{
  ros::Rate rate(10.0);
  tf::StampedTransform transform;
  try{
    listener->lookupTransform("vicon", "base",
			      ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d curPt(transform.getOrigin().x(), transform.getOrigin().y(), 0);
  tf::Matrix3x3 m(transform.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  geometry_msgs::Twist twist;
  
  while (curPt.distanceXY(goalPt) > 0.02 || abs(yaw - goalYaw) > DEG2RAD(3)){
    cout << "Errors: " << curPt.distanceXY(goalPt) << ", " << abs(yaw - goalYaw) << endl;
    
    try{
      listener->lookupTransform("vicon", "base",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    m = tf::Matrix3x3(transform.getRotation());
    
    curPt = point3d(transform.getOrigin().x(), transform.getOrigin().y(), 0);
    
    m.getRPY(roll, pitch, yaw);
    
    std::cout << "X: " << transform.getOrigin().getX() - goalPt.x() << ", Y: " << transform.getOrigin().getY() - goalPt.y() << ", Theta: " << yaw - goalYaw << std::endl;
    
    double cmdX = -sign(transform.getOrigin().x() - goalPt.x()) * min(0.3, 5 * abs(transform.getOrigin().x() - goalPt.x()));
    double cmdY = -sign(transform.getOrigin().y() - goalPt.y()) * min(0.3, 5 * abs(transform.getOrigin().y() - goalPt.y()));
    twist.linear.x = cmdX * cos(yaw) + cmdY * sin(yaw);
    twist.linear.y = -cmdX * sin(yaw) + cmdY * cos(yaw);
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = -sign(yaw - goalYaw) * min(0.3, 6 * abs(yaw - goalYaw));
    
    cmd_pub.publish<geometry_msgs::Twist>(twist);
    
    std::cout << "X: " << cmdX << std::endl;
    std::cout << "Y: " << cmdY << std::endl;
    std::cout << "Yaw Command: " << -sign(yaw - goalYaw) * std::min(0.3, 6 * std::abs(yaw - goalYaw)) << std::endl;

    rate.sleep();
  }	
  
  cout << "Stopping" << endl;
  
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  
  cmd_pub.publish<geometry_msgs::Twist>(twist);
  
  return;
}

void flag_cb(const std_msgs::Int32ConstPtr& msg)
{
  cout << "Starting Iteration " << msg->data << endl;
  startAlg = true;
}

void grid_cb(const nav_msgs::OccupancyGridConstPtr& grid)
{
  map2d = *grid;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_3d");
  
  ros::NodeHandle node;
  
  ros::Rate rate(10.0);
  
  tf::TransformListener listener;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::StampedTransform stransform;
  tf::Quaternion q;

  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("/youbotPath", 15);
  ros::Publisher grid_pub = node.advertise<nav_msgs::OccupancyGrid>("/nav_map", 15);
  ros::Publisher mark_pub = node.advertise<visualization_msgs::MarkerArray>("/nbv_list", 15);
  ros::Publisher nbv_pub = node.advertise<std_msgs::Float64MultiArray>("/nbv", 15);
  
  //sleep(5);
  
  ros::Subscriber sub = node.subscribe("/minnow/octomap_binary", 15, map_cb);
  ros::Subscriber flag_sub = node.subscribe("/nbv_iter", 15, flag_cb);
  ros::Subscriber grid_sub = node.subscribe("/move_base/global_costmap/costmap", 15, grid_cb);
  
  point3d sensorOffset(0.1, 0.3, 0);
  point3d firstPos;
  
  /*//Create transform from sensor to robot base
  transform.setOrigin( tf::Vector3(sensorOffset.x(), sensorOffset.y(), 0.0) );
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "xtion"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "baseGoal", "sensorGoal"));
  //-----------------------*/
  
  const clock_t start = std::clock();
  
  point3d curPose(-1.1, -0.05, 0.0);
  vector<point3d> NBVList;
  vector<point3d> PoseList;
  visualization_msgs::MarkerArray views;
  
  transform.setOrigin( tf::Vector3(curPose.x(), curPose.y(), 0) );
  q.setRPY(0, 0, DEG2RAD(-90));
  transform.setRotation(q);
  ros::Time gTime = ros::Time::now();
  br.sendTransform(tf::StampedTransform(transform, gTime, "vicon", "sensorGoal"));
  
  //Create transform from sensor to robot base
  //transform.setOrigin( tf::Vector3(sensorOffset.x(), sensorOffset.y(), 0.0) );
  //q.setRPY(0, 0, 0);
  //transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "xtion", "base"));
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "sensorGoal", "baseGoal"));
  //-----------------------
  
  sleep(1);
  
  //int x;
  //cin >> x;
  
  //moveBase(origin, yaw, &listener);
  
  cout << range << endl;
  
  cout << sRes << endl;
  
  //Setup Stuff
  //OcTree tree(res);
  
  cout << "Loading Sensor Model from File" << endl;
    
  ifstream readEm("SensorModel.bin", ios::in | ios::binary);
  
  while (!readEm.eof())
  {
    int buffer;
    readEm.read((char*)&buffer, sizeof(int));
    if (buffer < 0) break;
    optiMap.push_back(buffer);
    vector<int> tempRay;
    while (!readEm.eof())
    {
      readEm.read((char*)&buffer, sizeof(int));
      if (buffer < 0) break;
      tempRay.push_back(buffer);
    }
    optiRays.push_back(tempRay);
  }
  
  readEm.close();
  
  cout << "Finished Loading Sensor Model" << endl;
  
  double tempRoll, tempPitch, tempYaw;
  /*try{
  gTime = ros::Time::now();
  listener.waitForTransform("vicon", "xtion", gTime, ros::Duration(5.0));
  listener.lookupTransform("vicon", "xtion", gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d origin(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  tf::Matrix3x3 rot(stransform.getRotation());
  rot.getRPY(tempRoll, tempPitch, tempYaw);*/
  
  point3d origin(0, 0, 0);
  
  NBVList.push_back(origin);
  
  cout << "Starting Pose: " << tempYaw << " : " << tempPitch << endl;
  
  visualization_msgs::Marker mark;
  mark.header.frame_id = "map";
  mark.header.stamp = ros::Time::now();
  mark.ns = "basic_shapes";
  mark.id = 0;
  mark.type = visualization_msgs::Marker::ARROW;
  mark.action = visualization_msgs::Marker::ADD;
  
  mark.pose.position.x = origin.x();
  mark.pose.position.y = origin.y();
  mark.pose.position.z = origin.z();
  
  q.setRPY(0, 0, 0);

  mark.pose.orientation.x = q.getX();
  mark.pose.orientation.y = q.getY();
  mark.pose.orientation.z = q.getZ();
  mark.pose.orientation.w = q.getW();
  
  mark.scale.x = 0.2;
  mark.scale.y = 0.02;
  mark.scale.z = 0.02;
  
  mark.color.r = 1.0f;
  mark.color.g = 0.0f;
  mark.color.a = 1.0;
  mark.color.b = 0.0f;
  
  views.markers.push_back(mark);
  
  mark_pub.publish(views);
  
  int NBVcount = 0;
  while (ros::ok())
  {
    cout << "NBV " << NBVcount++ << endl;
    
    while (!startAlg && ros::ok())
    {
      ros::spinOnce();
    }
    
    cout << "Request Received" << endl;
    
    while ((map2d.data.size() == 0 || !mapReceived) && ros::ok())
    {
      ros::spinOnce();
    }
    
    grid_pub.publish(map2d);
    
    //AbstractOcTree
    
    OcTree* treeTemp = binaryMsgToMap(mapMsg);
    
    //OcTree treeTemp(0.01);
    
    treeTemp->writeBinary("firstMap.bt");
    
    cout << "Loaded the Map" << endl;
    
    /*try{
    gTime = ros::Time::now();
    listener.waitForTransform("vicon", "xtion", gTime, ros::Duration(5.0));
    listener.lookupTransform("vicon", "xtion", gTime, stransform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    point3d camPosition (stransform.getOrigin().x(), stransform.getOrigin().y(), 0);*/
    origin = NBVList.back();
    
    cout << "Map Loaded " << treeTemp->getResolution() << endl;
    
    point3d egoPose(0, 0, 0);
    point3d endPoint(-1.4, -1.5, 0);
    
    //pObj Tuning Coefficients
    double alpha = 0.5;
    double beta = 2.0;
    
    size_t pointCount = 0;
    
    //point3d origin = curPose;
    
    OcTree tree(res);
    
    tree.setClampingThresMax(0.999);
    tree.setClampingThresMin(0.001);
    tree.setProbMiss(0.1);
    tree.setProbHit(0.995);
    
    tree.readBinary("firstMap.bt");
    
    tree.expand();
    
    /*for (OcTree::leaf_iterator iter = tree.begin_leafs(); iter != tree.end_leafs(); iter++)
    {
      if (tree.isNodeOccupied(*iter)) && (iter.getZ() < 0.03 || iter.getX() < -2 || iter.getX() > 1 || iter.getY() < -1 || iter.getY() > 1))
      {
	tree.setNodeValue(iter.getKey(), -lEmpty);
	tree.updateNode(iter.getKey(), false);
	
	if (tree.isNodeOccupied(*iter))
	{
	  cout << "Errrnk" << endl;
	}
      }
    }*/
    
    cout << origin.x() << ", " << origin.y() << endl;
    
    origin = tree.keyToCoord(tree.coordToKey(origin));
    
    cout << "Origin: " << origin.x() << " : " << origin.y() << " : " << origin.z() << endl;
    
    if (NBVcount == 1)
    {
      firstPos = origin;
    }
    
    //Use Second Tree to store pObj entities for NBV prediction    
    OcTree tree2(tree);
    
    string fileName = "algMap" + to_string(NBVcount) + ".bt";
    
    tree.writeBinary(fileName);
    
    //-------------------
    //Cost Function Work - 2D Map Creation, Dijkstras Path
    //-------------------
      
    double robotRad = 0.3;
    
    /*nav_msgs::MapMetaData mapMeta = map2d.info;
    double minX = mapMeta.origin.position.x;
    double minY = mapMeta.origin.position.y;
    
    cout << "2D Res: " << mapMeta.resolution;
    
    double rangeX = mapMeta.width;
    double rangeY = mapMeta.height;
    
    vector<GridNode> gridMap(rangeX * rangeY, GridNode());
    
    for (int x = 0; x < rangeX; x++)
    {
      for (int y = 0; y < rangeY; y++)
      {
	//cout << "XY: " << x << ", " << y << endl;
	gridMap[rangeX * y + x].coords = point3d(double(x) * res + minX, double(y) * res + minY, 0);
	
	for (int i = -1; i <= 1; i += 1)
	{
	  for (int j = -1; j <= 1; j += 1)
	  {
	    if ((i != 0 || j != 0) && x + i >= 0 && x + i < rangeX && y + j >= 0 && y + j < rangeY)
	    {
	      gridMap[rangeX * y + x].neighbors.push_back(&gridMap[rangeX * (y + j) + x + i]);
	      gridMap[rangeX * y + x].edges.push_back(sqrt(pow(i, 2) + pow(j, 2)) * res);
	      
	      //cout << gridMap[rangeX*y + x].neighbors.size() << ", " << gridMap[rangeX*y + x].edges.size() << endl;
	    }
	  }
	}
      }
    }
    
    cout << "First" << endl;
    
    cout << "Relative Sizes: " << map2d.data.size() << " : " << gridMap.size() << endl;
    cout << "Dimensions: " << mapMeta.width << " by " << mapMeta.height << endl;
    
    for (int x = 0; x < rangeX; x++)
    {
      for (int y = 0; y < rangeY; y++)
      {	
	int index = rangeX * y + x;
	if (map2d.data[index] > 50)
	{
	  gridMap[index].object = true;
	  gridMap[index].occupied = true;
	  
	  for (double offX = -robotRad; offX <= robotRad; offX += res)
	  {
	    for (double offY = -robotRad; offY <= robotRad; offY += res)
	    {
	      if (sqrt(pow(offX, 2) + pow(offY, 2)) <= robotRad)
	      {
		int xInd = round((gridMap[index].coords.x() + offX - minX) * rFactor);
		int yInd = round((gridMap[index].coords.y() + offY - minY) * rFactor);
		
		if (xInd >= 0 && xInd < rangeX && yInd >= 0 && yInd < rangeY)
		{
		  int offIdx = yInd * rangeX + xInd;
		  
		  gridMap[offIdx].occupied = true;
		}
	      }
	    }
	  }
	}
      }
    }
    
    cout << "Second" << endl;
    
    cout << rangeX << ", " << rangeY << " : " << int((origin.x() - minX) * rFactor) << ", " << int((origin.y() - minY) * rFactor) << endl;
    
    int origIdx = rangeX * int((origin.y() - minY) / res) + int((origin.x() - minX) / res);
    
    //round(((origin.y() - minY) * rFactor) * rangeX + (origin.x() - minX) * rFactor);
    
    cout << origIdx << "     " << gridMap.size() << endl;
    
    GridNode* originNode = &gridMap[origIdx];
    originNode->cost = 0;
    
    cout << "Got Here" << endl;
    
    MinHeap heapy;
    
    heapy.Push(originNode);
    
    while (heapy.GetLength() > 0)
    {
      GridNode* minNode = heapy.Pop();
      
      vector<float>::iterator edgeIt = minNode->edges.begin();
      for (vector<GridNode*>::iterator iter = minNode->neighbors.begin(); iter != minNode->neighbors.end(); iter++, edgeIt++)
      {
	GridNode* nodePt = *iter;
	if ((*iter)->occupied == 0 && (*iter)->cost > (minNode->cost + *edgeIt))
	{
	  (*iter)->parent = minNode;
	  
	  if ((*iter)->state == 0)
	  {
	    (*iter)->cost = minNode->cost + *edgeIt;
	    heapy.Push((*iter));
	    (*iter)->state = 1;
	  }
	  else if ((*iter)->state == 1)
	  {
	    heapy.Update((*iter)->heapIdx, minNode->cost + *edgeIt);
	  }
	}
      }
    }
    
    cout << "Dijkstra Path Complete" << endl;
    
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";
    
    nav_msgs::MapMetaData metas;
    metas.resolution = res;
    metas.origin.position.x = minX;
    metas.origin.position.y = minY;
    metas.origin.position.z = 0.07;
    metas.height = rangeY;
    metas.width = rangeX;
    
    grid.info = metas;
    
    cout << gridMap.size() << endl;
    
    for (vector<GridNode>::iterator iter = gridMap.begin(); iter != gridMap.end(); iter++)
    {
      if (iter->occupied)
      {
	grid.data.push_back(100);
      }
      else
      {
	grid.data.push_back(0);
      }
    }
    
    grid_pub.publish(grid);
    
    cout << "Map Published" << endl;*/
    
    pointCount = 0;
    
    int cubeCount = 0;
    int emptyCount = 0;
    /*tree.expand();
    for (OcTree::leaf_iterator iter = tree.begin_leafs(); iter != tree.end_leafs(); iter++)
    {
      if (tree.isNodeOccupied(*iter))
      {
	pointCount++;
      }
      else
      {
	emptyCount++;
      }
    }*/
    
    cout << cubeCount << " cubes" << endl;
    
    cout << "Empties: " << emptyCount << endl;
    
    //OcTreeCustom infoTree(res);
    
    //infoTree.readBinary("kinect.bt");
    
    //Copy all occupancy values from known nodes into pObj variables for those nodes
    
    //infoTree.expand();
    
    /*for (OcTreeCustom::iterator iter = infoTree.begin(); iter != infoTree.end(); iter++)
    *   { *
    *   iter->setpObj(float(iter->getOccupancy()));
  }*/
    
    int cellCount = 0;
    int unkCount = 0;
    
    //pointCount = 0;
    
    cout << "Beginning pObj Processing" << endl;
    
    for (OcTree::leaf_iterator iter = tree.begin_leafs(); iter != tree.end_leafs(); iter++)
    {
      OcTreeKey occKey = iter.getKey();
      OcTreeKey unKey;
      OcTreeKey freeKey;
      OcTreeNode *cellNode;
      OcTreeNode *unNode;
      
      if (tree.isNodeOccupied(*iter))
      {
	cellCount++;
	
	for (int i = -1; i <= 1; i++)
	{
	  for (int j = -1; j <= 1; j++)
	  {
	    for (int k = -1; k <= 1; k++)
	    {
	      if (i != 0 || j != 0 || k != 0)
	      {
		unKey = occKey;
		unKey[0] += i;
		unKey[1] += j;
		unKey[2] += k;
		
		unNode = tree.search(unKey);
		
		if (unNode == NULL)
		{
		  bool critical = false;
		  for (int u = -1; u <= 1; u++)
		  {
		    for (int v = -1; v <= 1; v++)
		    {
		      for (int w = -1; w <= 1; w++)
		      {
			if (abs(u) + abs(v) + abs(w) == 1)
			{
			  freeKey = unKey;
			  freeKey[0] += u;
			  freeKey[1] += v;
			  freeKey[2] += w;
			  
			  cellNode = tree.search(freeKey);
			  
			  if (cellNode != NULL && !tree.isNodeOccupied(cellNode) && tree.keyToCoord(freeKey).z() > 0.15)
			  {//unKey is a critical unknown cell, proceed to update pObj of unknown cells in vicinity
			    critical = true;
			  }
			}
		      }
		    }
		  }
		  
		  if (critical)
		  {
		    //cout << "Occ: " << tree.keyToCoord(occKey).x() << ", " << tree.keyToCoord(occKey).y() << ", " << tree.keyToCoord(occKey).z() << endl;
		    //cout << "Unk: " << tree.keyToCoord(unKey).x() << ", " << tree.keyToCoord(unKey).y() << ", " << tree.keyToCoord(unKey).z() << endl;
		    tree2.updateNode(tree2.coordToKey(tree.keyToCoord(unKey)), (float)log(alpha / (1 - alpha)));
		    
		    //New method using custom tree
		    //infoTree.updateNode(tree.keyToCoord(unKey), float(0));
		    //infoTree.search(tree.keyToCoord(unKey))->setpObj((float)log(alpha / (1 - alpha)));
		    
		    unkCount++;
		    
		    for (double boxX = -5 * res; boxX <= 5 * res; boxX += res)
		    {
		      for (double boxY = -5 * res; boxY <= 5 * res; boxY += res)
		      {
			for (double boxZ = -5 * res; boxZ <= 5 * res; boxZ += res)
			{
			  point3d objPoint = tree.keyToCoord(unKey) + point3d(boxX, boxY, boxZ);
			  if (objPoint.z() > 0.05)
			  {
			    double pObj = alpha*exp(-sqrt(pow(boxX, 2) + pow(boxY, 2) + pow(boxZ, 2)) * beta);
			    cellNode = tree.search(objPoint);
			    if (cellNode == NULL)
			    {
			      if (tree2.search(objPoint) == NULL)
			      {
				tree2.setNodeValue(objPoint, log(pObj / (1 - pObj)));
				
				pointCount++;
			      }
			      else
			      {
				if ((tree2.search(objPoint))->getLogOdds() < (float)log(pObj / (1 - pObj)))
				{
				  tree2.setNodeValue(objPoint, log(pObj / (1 - pObj)));
				}
				//tree2.updateNode(objPoint, max((tree2.search(objPoint))->getLogOdds(), (float)log2(pObj / (1 - pObj))));
			      }
			    }
			    /*else
			    *											{
			    *												tree2.setNodeValue(objPoint, cellNode->getLogOdds());
			    * 
			    *												infoTree.search(objPoint)->setpObj(cellNode->getOccupancy());
			  }*/
			  }
			}
		      }
		    }
		  }
		}
	      }
	    }
	  }
	}
      }
    }
    
    /*infoTree.expand();
    * 
    *   f *or (OcTreeCustom::leaf_iterator iter = infoTree.begin_leafs(searchDepth); iter != infoTree.end_leafs(); iter++)
    *   {
    *   iter->updateOccupancyChildren();
    *   if (iter->getLogOdds() != 0 && abs(iter->getOccupancy() - iter->getpObj()) > 0.01)
    *   {
    *   cout << iter->getLogOdds() << ", " << iter->getOccupancy() << ", " << iter->getpObj() << endl;
    *   addCube(iter.getCoordinate(), res, 0, 0, 1, 0.5);
  }
  if (iter->getLogOdds() == 0 && iter->getpObj() > 0.001)
  {
  addCube(iter.getCoordinate(), sRes, 0, 1, 0, 0.5);
  }
  }*/
    
    /*tree.expand();
    tree2.expand();
    for (OcTree::leaf_iterator iter = tree.begin_leafs(); iter != tree.end_leafs(); iter++)
    {
      if (tree2.search(iter.getCoordinate())->getOccupancy() != iter->getOccupancy())
      {
	cout << tree2.search(iter.getCoordinate())->getOccupancy() << ", " << iter->getOccupancy() << endl;
      }
    }*/
    
    cout << "Nothing Different" << endl;
    
    tree2.writeBinary("objMap.bt");
    
    pointCount = 0;
    
    tree2.expand();
    
    KeyRay cellList;
    point3d egoCOG(2.95, 1.95, 0);
    Quaternion egoTheta(point3d(0, 0, -M_PI / 2));
    endPoint = point3d(3.0, 0, 0);
    OcTreeKey myCell;
    OcTreeNode *cellNode;
    double cellLike;
    
    Pose6D trialPose(egoCOG, egoTheta);
    endPoint = trialPose.transform(endPoint);
    
    //Pre-Computing Table Stuff
    point3d rayOrigin(0.005, 0.005, 0.005);
    point3d rayInit(nRayLength * res, 0, 0);
    point3d rayEnd;
    
    /*vector<int> castMap;
    *   v *ector<int> logAdd;
    *   
    *   cellCount = 0;
    *   int traversed = 0;
    *   
    *   Pointcloud castCloud;
    *   
    *   for (double yaw = 0; yaw < 360; yaw++)
    *   {
    *   cout << yaw << endl;
    *   for (double pitch = -90; pitch <= 90; pitch++)
    *   {
    *   Quaternion rayTheta(point3d(0, DEG2RAD(pitch), DEG2RAD(yaw)));
    *   Pose6D rayPose(rayOrigin, rayTheta);
    *   rayEnd = rayPose.transform(rayInit);
    *   
    *   cellList.reset();
    *   tree.computeRayKeys(rayOrigin, rayEnd, cellList);
    *   //cout << "cellList Size: " << cellList.size() << endl;
    *   for (KeyRay::iterator iter = ++cellList.begin(); iter != cellList.end(); iter++)
    *   {
    *   traversed++;
    *   point3d rayCoord = tree.keyToCoord(*iter) - rayOrigin;
    *   
    *   if (rayCoord.norm() * sRes / res > 0.6)
    *   {
    *   int index = round(rayCoord.x() / res) + nRayLength + (2 * nRayLength + 1) * round(rayCoord.y() / res + nRayLength) + pow(2 * nRayLength + 1, 2) * round(rayCoord.z() / res + nRayLength);
    *   vector<int>::iterator findIt = find(castMap.begin(), castMap.end(), index);
    *   if (findIt == castMap.end())
    *   {
    *   castMap.push_back(index);
    *   castCloud.push_back(rayCoord + rayOrigin);
    *   logAdd.push_back(lFilled);
    *   cellCount++;
  }
  else
  {
  int foundIndex = distance(castMap.begin(), findIt);
  logAdd[foundIndex] += lFilled;
  }
  }
  }
  }
  }
  cout << "Cells in CastMap: " << cellCount << "out of " << traversed << "cells traversed" << endl;
  cout << "Min Index: " << *min_element(castMap.begin(), castMap.end()) << endl;
  cout << "Max Index: " << *max_element(castMap.begin(), castMap.end()) << endl;
  cout << "Max logAdd: " << *max_element(logAdd.begin(), logAdd.end()) << endl;
  cout << nRayLength << endl;
  cout << "Cloud Size: " << castCloud.size() << endl;
  cout << "Inserting Cloud..." << endl;

  tree.insertPointCloud(castCloud, rayOrigin);
  tree.writeBinary("castMap.bt");

  //Write pre-computed tables to files for quick retrieval
  ofstream outFile("mapCast.bin", ios::out | ios::binary);
  for (vector<int>::iterator iter = castMap.begin(); iter != castMap.end(); iter++)
  {
  outFile.write((char*)&(*iter), sizeof(int));
  }
  outFile.close();

  outFile = ofstream("logAdd.bin", ios::out | ios::binary);
  for (vector<int>::iterator iter = logAdd.begin(); iter != logAdd.end(); iter++)
  {
  outFile.write((char*)&(*iter), sizeof(int));
  }
  outFile.close();

  cout << "Done Writing" << endl;

  while (true)
  {

  }*/
    
    //--------------------
    //Create 3D Arrays of object information for quick retrieval in infoGain calcs
    //--------------------
    
    //First pass to find limits of info bounding box	
    double temp_x, temp_y, temp_z;
    tree2.getMetricMax(temp_x, temp_y, temp_z);
    //infoTree.getMetricMin(temp_x, temp_y, temp_z);
    ibxMin = point3d(temp_x, temp_y, temp_z);
    tree2.getMetricMin(temp_x, temp_y, temp_z);
    //infoTree.getMetricMax(temp_x, temp_y, temp_z);
    ibxMax = point3d(temp_x, temp_y, temp_z);
    
    for (OcTree::iterator iter = tree2.begin_leafs(searchDepth); iter != tree2.end_leafs(); iter++)
    {
      if (iter->getOccupancy() > 0.001)
      {
	//addCube(iter.getCoordinate(), sRes, 0, 0, 1, 0.5);
	ibxMin.x() = min(ibxMin.x(), iter.getCoordinate().x());
	ibxMin.y() = min(ibxMin.y(), iter.getCoordinate().y());
	ibxMin.z() = min(ibxMin.z(), iter.getCoordinate().z());
	
	ibxMax.x() = max(ibxMax.x(), iter.getCoordinate().x());
	ibxMax.y() = max(ibxMax.y(), iter.getCoordinate().y());
	ibxMax.z() = max(ibxMax.z(), iter.getCoordinate().z());
      }
    }
    
    /*for (OcTreeCustom::leaf_iterator iter = infoTree.begin_leafs(searchDepth); iter != infoTree.end_leafs(); iter++)
    *   { *
    *   iter->updateOccupancyChildren();
    *   if (iter->getpObj() > 0.001)
    *   {
    *   if (iter.getCoordinate().x() < ibxMin.x()) ibxMin.x() = iter.getCoordinate().x();
    *   if (iter.getCoordinate().y() < ibxMin.y()) ibxMin.y() = iter.getCoordinate().y();
    *   if (iter.getCoordinate().z() < ibxMin.z()) ibxMin.z() = iter.getCoordinate().z();
    *   
    *   if (iter.getCoordinate().x() > ibxMax.x()) ibxMax.x() = iter.getCoordinate().x();
    *   if (iter.getCoordinate().y() > ibxMax.y()) ibxMax.y() = iter.getCoordinate().y();
    *   if (iter.getCoordinate().z() > ibxMax.z()) ibxMax.z() = iter.getCoordinate().z();
  }
  }*/
    
    cout << "Min Corner: " << ibxMin.x() << ", " << ibxMin.y() << ", " << ibxMin.z() << endl;
    cout << "Max Corner: " << ibxMax.x() << ", " << ibxMax.y() << ", " << ibxMax.z() << endl;
    
    cout << "S: " << sFactor << endl;
    
    //Determine required size of infoChunk array and initialize accordingly
    boxX = round((ibxMax.x() - ibxMin.x()) / sRes + 1);
    boxY = round((ibxMax.y() - ibxMin.y()) / sRes + 1);
    boxZ = round((ibxMax.z() - ibxMin.z()) / sRes + 1);
    
    cout << boxX << ", " << boxY << ", " << boxZ << endl;
    
    infoChunk = vector<vector<float> >(boxX * boxY * boxZ, { 0 , 0 });
    
    //Second pass to populate 3D array with info values
    tree2.expand();
    for (OcTree::leaf_bbx_iterator iter = tree2.begin_leafs_bbx(ibxMin, ibxMax, searchDepth); iter != tree2.end_leafs_bbx(); iter++)
    {
      if (iter.getCoordinate().z() > 0.05 && iter->getOccupancy() > 0.001)
      {
	int xInd = round((iter.getCoordinate().x() - ibxMin.x()) * sFactor);
	int yInd = round((iter.getCoordinate().y() - ibxMin.y()) * sFactor);
	int zInd = round((iter.getCoordinate().z() - ibxMin.z()) * sFactor);
	
	if (xInd < 0 || xInd >= boxX || yInd < 0 || yInd >= boxY || zInd < 0 || zInd >= boxZ) continue;
	
	int index = xInd + yInd * boxX + zInd * boxX * boxY;
	
	//addCube(iter.getCoordinate(), sRes, 0, 0, 1, 0.5);
	infoChunk[index][1] = iter->getOccupancy();
	infoChunk[index][0] = iter->getOccupancy();
      }
    }
    
    tree.expand();
    for (OcTree::leaf_bbx_iterator iter = tree.begin_leafs_bbx(ibxMin, ibxMax, searchDepth); iter != tree.end_leafs_bbx(); iter++)
    {
      int xInd = round((iter.getCoordinate().x() - ibxMin.x()) * sFactor);
      int yInd = round((iter.getCoordinate().y() - ibxMin.y()) * sFactor);
      int zInd = round((iter.getCoordinate().z() - ibxMin.z()) * sFactor);
      
      if (xInd < 0 || xInd >= boxX || yInd < 0 || yInd >= boxY || zInd < 0 || zInd >= boxZ) continue;
      
      int index = xInd + yInd * boxX + zInd * boxX * boxY;
      
      if (iter.getCoordinate().z() > 0.05 && tree.isNodeOccupied(*iter))
      {
	double p = exp(iter->getLogOdds()) / (1 + exp(iter->getLogOdds()));
	infoChunk[index][0] = (-p * log2(p) - (1 - p) * log2(1 - p)) * infoChunk[index][1];
	//infoChunk[index][0] = iter->getLogOdds();
      }
      else
      {
	infoChunk[index][0] = 0;
      }
    }
    
    /*for (OcTreeCustom::leaf_bbx_iterator iter = infoTree.begin_leafs_bbx(ibxMin, ibxMax, searchDepth); iter != infoTree.end_leafs_bbx(); iter++)
    *   { *
    *   iter->updateOccupancyChildren();
    *   if (iter->getpObj() > 0.001)
    *   {
    *   int xInd = round((iter.getCoordinate().x() - ibxMin.x()) / sRes);
    *   int yInd = round((iter.getCoordinate().y() - ibxMin.y()) / sRes);
    *   int zInd = round((iter.getCoordinate().z() - ibxMin.z()) / sRes);
    *   
    *   cout << iter->getLogOdds() << endl;
    *   infoChunk[xInd + yInd * boxX + zInd * boxX * boxY][0] = iter->getLogOdds();
    *   infoChunk[xInd + yInd * boxX + zInd * boxX * boxY][1] = iter->getpObj();
  }
  }*/
    
    cout << "infoChunk populated, uploading sensor model" << endl;
    
    cout << "Begin Test" << endl;
    
    //Search Algorithm
    vector<InfoNode> gains;
    float maxHeight = 0.62;
    float minHeight = 0.28;
    
    pointCount = 0;
    
    double maxIG = 0;
    double maxCost = 0;
    double maxPitch = 0;
    double maxYaw = 0;
    point3d bestPos(0, 0, 0);
    int viewCount = 0;
    
    int searchCount = 0;
    
    point3d searchBoxMin = ibxMin - point3d(1.5, 1.5, 1.5);
    point3d searchBoxMax = ibxMax + point3d(1.5, 1.5, 1.5);
    
    searchBoxMin.z() = max(searchBoxMin.z(), minHeight);
    searchBoxMax.z() = min(searchBoxMax.z(), maxHeight);
    
    cout << "Search Space: " << searchBoxMin.x() << ", " << searchBoxMin.y() << ", " << searchBoxMin.z() << endl;
    cout << "To: " << searchBoxMax.x() << ", " << searchBoxMax.y() << ", " << searchBoxMax.z() << endl;
    
    //-------------
    //Heuristic Search Method
    //-------------
    /*const clock_t searchStart = clock();
    * 
    *   i *nt neighbDist = 2;
    *   double neighborStepSize = 0.02;
    *   vector<InfoNode> neighbors(6 * neighbDist, InfoNode());
    *   
    *   tree.expand();
    *   double xMax, yMax, zMax, xMin, yMin, zMin;
    *   tree.getMetricMax(xMax, yMax, zMax);
    *   tree.getMetricMin(xMin, yMin, zMin);
    *   for (OcTree::leaf_bbx_iterator iter = tree.begin_leafs_bbx(searchBoxMin, searchBoxMax, 10); iter != tree.end_leafs_bbx(); iter++)
    *   {
    *   if (iter.getCoordinate().z() < 0 || iter.getCoordinate().x() < xMin || iter.getCoordinate().x() > xMax || iter.getCoordinate().y() < yMin || iter.getCoordinate().y() > yMax || iter.getCoordinate().z() < zMin || iter.getCoordinate().z() > zMax || tree.search(iter.getCoordinate()) == NULL || tree.isNodeOccupied(tree.search(iter.getCoordinate()))) continue;
    *   
    *   searchCount++;
  }

  cout << searchCount << " positions to calculate" << endl;
  for (OcTree::leaf_iterator iter = tree.begin_leafs(11); iter != tree.end_leafs(); iter++)
  {
  if (iter.getCoordinate().z() < 0 || iter.getCoordinate().x() < xMin || iter.getCoordinate().x() > xMax || iter.getCoordinate().y() < yMin || iter.getCoordinate().y() > yMax || iter.getCoordinate().z() < zMin || iter.getCoordinate().z() > zMax || tree.search(iter.getCoordinate()) == NULL || tree.isNodeOccupied(tree.search(iter.getCoordinate()))) continue;

  if (viewCount % 100 == 0)
  {
  cout << viewCount << endl;
  }
  point3d nextPos = iter.getCoordinate();
  float pitch, yaw;
  double infoGain = getInfoGain(&tree, &tree2, nextPos, &yaw, &pitch);

  if (infoGain > 0)
  {
  gains.push_back(InfoNode(infoGain, 0, nextPos, yaw, pitch));

  if (infoGain > maxIG)
  {
  maxIG = infoGain;
  bestPos = nextPos;
  maxYaw = yaw;
  maxPitch = pitch;
  }

  //addCube(nextPos, 0.05, 1 - (infoGain / 40), infoGain / 40, 0, 0.9);
  }

  viewCount++;
  }

  cout << "First Pass: " << viewCount << " views" << endl;

  sort(gains.begin(), gains.end());

  vector<InfoNode>::iterator testIt = gains.end()--;

  vector<InfoNode> searchTails;

  vector<InfoNode>::iterator iter = gains.end()--;
  for (int i = 0; i < 5; i++)
  {
  cout << i << endl;
  InfoNode curNode = *iter;

  while (true)
  {
  cout << ".";
  point3d curPoint = curNode.coords;
  double curGain = curNode.infoGain;
  vector<InfoNode>::iterator neighbIt = neighbors.begin();

  for (int offX = -neighbDist; offX <= neighbDist; offX++)
  {
  for (int offY = -neighbDist; offY <= neighbDist; offY++)
  {
  for (int offZ = -neighbDist; offZ <= neighbDist; offZ++)
  {
  if ((offX != 0) + (offY != 0) + (offZ != 0) != 1) continue;

  point3d neighbor = curPoint + point3d(offX, offY, offZ) * neighborStepSize;
  neighbIt->coords = neighbor;

  if (neighbor.z() < 0 || neighbor.x() < xMin || neighbor.x() > xMax || neighbor.y() < yMin || neighbor.y() > yMax || neighbor.z() < zMin || neighbor.z() > zMax || tree.search(neighbor) == NULL || tree.isNodeOccupied(tree.search(neighbor)))
  {
  neighbIt->infoGain = 0;
  }
  else
  {
  float pitch, yaw;
  neighbIt->infoGain = getInfoGain(&tree, &tree2, neighbor, &yaw, &pitch);
  viewCount++;
  neighbIt->pitch = pitch;
  neighbIt->yaw = yaw;
  }

  neighbIt++;
  }
  }
  }

  InfoNode maxNeighb = *max_element(neighbors.begin(), neighbors.end());

  if (maxNeighb.infoGain <= curGain)
  {
  break;
  }
  else
  {
  curNode = maxNeighb;
  }
  }

  searchTails.push_back(curNode);

  iter--;

  cout << endl;
  }

  InfoNode hNBV = *max_element(searchTails.begin(), searchTails.end());

  const clock_t searchEnd = clock();

  cout << "Heuristic NBV: " << hNBV.coords.x() << ", " << hNBV.coords.y() << ", " << hNBV.coords.z() << endl;
  cout << "With Info Gain = " << hNBV.infoGain << endl;
  cout << "In " << double(searchEnd - searchStart) / CLOCKS_PER_SEC << " seconds, for " << viewCount << "candidate viewpoints" << endl;*/
    
 //--------------------------------------
 //Module 2D Map Search
 
 time_t time1 = time(0);
 viewCount = 0;
 
 nav_msgs::MapMetaData mapMeta = map2d.info;
 double minX = mapMeta.origin.position.x;
 double minY = mapMeta.origin.position.y;
 
 double mapRes = mapMeta.resolution;
 cout << "2D Res: " << mapRes;
 
 double rangeX = mapMeta.width;
 double rangeY = mapMeta.height;
 
 for (int x = 0; x < rangeX; x++)
 {
   for (int y = 0; y < rangeY; y++)
   {
     int index = rangeX * y + x;
     if (map2d.data[index] < 50)
     {
       float yaw, pitch;
       point3d coords(double(x) * mapRes + minX, double(y) * mapRes + minY, camHeight);
       if (tree.search(coords) == NULL || tree.isNodeOccupied(tree.search(coords))) continue;
       double infoGain = getInfoGain(&tree, &tree2, coords, &yaw, &pitch);
       viewCount++;
       
       gains.push_back(InfoNode(infoGain, 0, coords, yaw, pitch));
       
       if (infoGain > maxIG)
       {
	 maxIG = infoGain;
	 bestPos = coords;
	 maxYaw = yaw;
	 maxPitch = pitch;
       }
     }
   }
 }
 
 //--------------------------------------
    
    cout << "Time for " << viewCount << " views: " << (int)time(0) - time1 << endl;
    cout << "The NBV is at (" << bestPos.x() << ", " << bestPos.y() << ", " << bestPos.z() << ") with an expected info gain of " << maxIG << endl;
    
    cout << maxYaw << ", " << maxPitch << endl;
    
    cout << "Max Cost: " << maxCost << endl;
    
    point3d NBV = bestPos;
    
    point3d oldBest = bestPos;
    
    if (maxIG < 10)
    {
      cout << "Algorithm Complete" << endl;
      break;
    }
    
    NBVList.push_back(NBV);
    visualization_msgs::Marker mark;
    mark.header.frame_id = "map";
    mark.header.stamp = ros::Time::now();
    mark.ns = "basic_shapes";
    mark.id = NBVcount;
    mark.type = visualization_msgs::Marker::ARROW;
    mark.action = visualization_msgs::Marker::ADD;
    
    mark.pose.position.x = NBV.x();
    mark.pose.position.y = NBV.y();
    mark.pose.position.z = NBV.z();
    
    q.setRPY(0, DEG2RAD(-maxPitch), DEG2RAD(maxYaw));

    mark.pose.orientation.x = q.getX();
    mark.pose.orientation.y = q.getY();
    mark.pose.orientation.z = q.getZ();
    mark.pose.orientation.w = q.getW();
    
    mark.scale.x = 0.2;
    mark.scale.y = 0.02;
    mark.scale.z = 0.02;
    
    mark.color.r = 0.0f;
    mark.color.g = 1.0f;
    mark.color.a = 1.0;
    mark.color.b = 0.0f;
    
    views.markers.push_back(mark);
    
    mark_pub.publish(views);
    std_msgs::Float64MultiArray nbv_msg;
    nbv_msg.data = {NBV.x(), NBV.y(), DEG2RAD(maxYaw)};
    nbv_pub.publish(nbv_msg);
    
    startAlg = false;
    
//     while (ros::ok())
//     {
//       mark_pub.publish(views);
//       rate.sleep();
//       ros::spinOnce();
//     }
  }
  
  return 0;
}