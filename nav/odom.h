/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file odom.h
 **/

#pragma once
namespace cubot {

typedef struct Odometry {
  float x;
  float y;
  float angle;
  float vx;
  float vy;
  float w;
  float uwb_x;
  float uwb_y;
  float uwb_angle;
} Odom;

}
