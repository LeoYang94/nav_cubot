/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file main.cc
 **/
#include "speed_command.h"

#include "ros/ros.h"
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "speed_command");
  double x = std::atof(argv[1]);
  double y = std::atof(argv[2]);
  double z = std::atof(argv[3]);
  SpeedCommand speed_cmd(x, y, z);
}