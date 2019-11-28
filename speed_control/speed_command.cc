/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file speed_command.cc
 **/
#include "speed_command.h"

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

SpeedCommand::SpeedCommand(const double linear_x, const double linear_y,
                           const double angular_z) {
  SendSpeed(linear_x, linear_y, angular_z);
}

SpeedCommand::~SpeedCommand() {}

void SpeedCommand::SendSpeed(const double linear_x, const double linear_y,
                             const double angular_z) {
  ros::NodeHandle n;
  ros::Publisher pub_speed = n.advertise<geometry_msgs::Twist>(
      "speed_cmd", 10); //发布odom消息,odom前面不需加/;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;
    pub_speed.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
