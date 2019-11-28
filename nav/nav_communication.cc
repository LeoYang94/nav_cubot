/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file nav_communication.cc
 **/

#include "nav_communication.h"

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <vector>

namespace cubot {
NavCommunication::NavCommunication(const ros::NodeHandle &n,
                                   const ros::Publisher &pub_odom,
                                   const ros::Publisher &pub_imu)
    : n_(n), pub_odom_(pub_odom), pub_imu_(pub_imu) {
  ROS_INFO_STREAM("Start initial serial port..");
  ROS_ASSERT(SetSerial(&ser_));
  ROS_INFO_STREAM("Initilization complete");
  sub_ =
      n_.subscribe("/cubot/cmd_vel", 10, &NavCommunication::SendCmdVel,
                   this); // 10：不想等待太多，只想要最新的消息，所以取较小的数
}

bool NavCommunication::ReadAndPubOdom() {
  std::vector<uint8_t> odom_raw;
  const size_t kOdomDataSize = 29;
  Odom odom = {0.0, 0.0, 0.0, //声明里程计结构体
               0.0, 0.0, 0.0};
  //ros::Rate loop_rate(10);
  while (ros::ok()) {
    ser_.read(odom_raw);
    //判断帧头
    if (odom_raw.size() == 4 &&
        (odom_raw[0] != 0xaa || odom_raw[1] != 0xaa || odom_raw[2] != 0xf1)) {
      ROS_INFO_STREAM("Read odom raw data failed");
      odom_raw.clear();
      continue;
    }
    if (odom_raw.size() == kOdomDataSize) {
      //ROS_INFO_STREAM("Read odom raw data success");
      if (!GetFilterOdomData(odom_raw, &odom)) {
        continue;
      }
      ros::Time current_time = ros::Time::now();
      PublishOdom(odom, current_time);
      PublishTf(odom, current_time);
    }
    ros::spinOnce();
    //loop_rate.sleep();
  }
  //每一次读串口循环结束后，进入回调函数下发控制指令
}

bool NavCommunication::SetSerial(serial::Serial *const ser) const {
  ser->setPort(
      "/dev/usart_nav");    // usart_nav串口号(将电脑上的某个固定的usb口重命名为usart,放弃使用易变的/dev/ttyUSB*)
  ser->setBaudrate(115200); //波特率
  serial::Timeout to =
      serial::Timeout::simpleTimeout(1000); //字节间读写超时设为1s
  ser->setTimeout(to);                      //设置串口超时
  ser->open();                              //打开串口
  return ser->isOpen();
}

void NavCommunication::SendCmdVel(const geometry_msgs::Twist cmd_vel) const {
  std::cout << "got the cmd_vel,sending it.." << std::endl;
  std::string first_str(1, 0xaa);
  std::string second_str(1, 0xf1);
  std::string send_data = first_str + second_str;
  std::cout << "cmd_vel->x:" << cmd_vel.linear.x << std::endl;
  std::cout << "cmd_vel->y:" << cmd_vel.linear.y << std::endl;
  std::cout << "cmd_vel->z:" << cmd_vel.angular.z << std::endl;
  static uint8_t i = 0;
  i = 0;
  while (i < 8) {
    send_data += (*((char *)&cmd_vel.linear.x + (i++)));
  }
  i = 0;
  while (i < 8) {
    send_data += (*((char *)&cmd_vel.linear.y + (i++)));
  }
  i = 0;
  while (i < 8) {
    send_data += (*((char *)&cmd_vel.angular.z + (i++)));
  }
  char sum = 0;
  for (uint8_t i = 0; i < send_data.size(); i++)
    sum += char(send_data[i]);
  send_data += sum;
  static size_t number = 0;
  number = ser_.write(send_data);
  ROS_INFO_STREAM("Write cmd vel data!");
  std::cout << "number" << number << std::endl;
}

bool NavCommunication::GetFilterOdomData(std::vector<uint8_t> &odom_raw,
                                         cubot::Odom *const odom) const {
  union CharToFloat {
    uint8_t hex[32];      // 32个uint8_t
    float odom_serial[8]; //长度等于8个float
  } c2f;
  uint8_t sum = 0;
  for (uint8_t j = 0; j < odom_raw.size() - 1; j++) {
    sum += odom_raw[j];
  }
  if (odom_raw.back() != sum) {
    odom_raw.clear();
    return false;
  } else {
    //存储为uint8_t型
    for (uint8_t i = 0; i < odom_raw.size(); i++) {
      c2f.hex[i] = odom_raw[i];
    }
    //还原为float
    odom->vx = c2f.odom_serial[1];
    odom->vy = c2f.odom_serial[2];
    odom->w = c2f.odom_serial[3];
    odom->x = c2f.odom_serial[4];
    odom->y = c2f.odom_serial[5];
    odom->angle = c2f.odom_serial[6];
    odom_raw.clear();
   // std::cout << "odom.x: " << odom->x << std::endl;
   // std::cout << "odom.y: " << odom->y << std::endl;
   // std::cout << "odom.angle: " << odom->angle << std::endl;
    //中值滤波过滤跳变
    // MiddleFilter(odom);
  }
  return true;
}

bool NavCommunication::MiddleFilter(cubot::Odom *const odom) const {
  static int num = 0;
  static std::vector<std::vector<float> > container;
  Odom odom_out = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  num = (num + 1) % 3;
  container.resize(6);
  if (container[0].size() < 3) {
    for (int i = 0; i < 6; i++)
      container[i].push_back(*((float *)odom + i));
    return true;
  } else {
    for (int i = 0; i < 6; i++) {
      container[i][num] = *((float *)odom + i);
      sort(container[i].begin(), container[i].end());
      *((float *)&odom_out + i) = container[i][1];
      *odom = odom_out;
    }
    return true;
  }
}

void NavCommunication::PublishOdom(const cubot::Odom &odom,
                                   const ros::Time &current_time) const {
  nav_msgs::Odometry odom_pub;
  odom_pub.header.stamp = current_time; //本消息的时间戳
  odom_pub.header.frame_id = "odom"; // 消息里的frame_id表示这些消息的参考原点
  odom_pub.child_frame_id = "base_footprint";
  odom_pub.pose.pose.position.x = odom.x;
  odom_pub.pose.pose.position.y = odom.y;
  odom_pub.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion odom_quaternion =
      tf::createQuaternionMsgFromYaw(odom.angle); //将角度（弧度制）转换为四元数
  odom_pub.pose.pose.orientation = odom_quaternion;
  pub_odom_.publish(odom_pub);
}

void NavCommunication::PublishTf(const cubot::Odom &odom,
                                 const ros::Time &current_time) const {
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.transform.translation.x = odom.x;
  odom_trans.transform.translation.y = odom.y;
  odom_trans.transform.translation.z = 0.0;
  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(odom.angle);
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster_.sendTransform(odom_trans);
}
}
