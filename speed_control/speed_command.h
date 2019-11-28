/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file speed_command.h
 **/
#include <iostream>

class SpeedCommand{
public:
  SpeedCommand(const double linear_x, const double linear_y,
               const double angular_z);
  ~SpeedCommand();

private:
  void SendSpeed(const double linear_x, const double linear_y,
                 const double angular_z);
};