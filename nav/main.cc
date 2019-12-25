/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file main.cc
 **/

#include "nav_communication.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "communication");
  ros::NodeHandle n;
  ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>(
      "odom", 10); //发布odom消息,odom前面不需加/;
  ros::Publisher pub_uwb = n.advertise<nav_msgs::Odometry>(
      "uwb_pose", 10); //发布odom消息,odom前面不需加/;
  ros::Publisher pub_imu =
      n.advertise<sensor_msgs::Imu>("imu", 10); //发布imu消息
  cubot::NavCommunication nav_communication(n, pub_odom, pub_uwb, pub_imu);
  nav_communication.ReadAndPubOdom();
  return 0;
}
