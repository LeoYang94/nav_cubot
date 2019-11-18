/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file imu.h
 **/

#pragma once

namespace cubot {
typedef struct Imu_message {
  float roll;
  float pitch;
  float yaw;
  float vroll;
  float vpitch;
  float vyaw;
  float ax;
  float ay;
  float az;
} IMU;
}
