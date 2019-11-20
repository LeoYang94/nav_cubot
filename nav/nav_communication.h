/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file nav_communication.h
 **/

#pragma once
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <vector>

#include "imu.h"
#include "odom.h"

namespace cubot {
class NavCommunication {
public:
  explicit NavCommunication(const ros::NodeHandle &n,
                            const ros::Publisher &pub_odom,
                            const ros::Publisher &pub_imu);
  bool ReadAndPubOdom();

private:
  bool SetSerial(serial::Serial *const ser) const;
  void SendCmdVel(const geometry_msgs::Twist cmd_vel) const;
  bool GetFilterOdomData(std::vector<uint8_t> &odom_raw,
                         Odom *const odom) const;
  bool MiddleFilter(Odom *const odom) const;
  void PublishOdom(const Odom &odom, const ros::Time& current_time) const;
  void PublishTf(const Odom &odom, const ros::Time& current_time)const;

private:
  mutable serial::Serial ser_;
  ros::NodeHandle n_;
  ros::Publisher pub_odom_;                   // odom topic 发布器
  ros::Publisher pub_imu_;                    // imu topic 发布器
  ros::Subscriber sub_;                       // cmd_vel订阅器
  mutable tf::TransformBroadcaster odom_broadcaster_; // odom tf发布器
};
}
