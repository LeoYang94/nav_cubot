#!/bin/bash

echo "remap the device serial port(ttyUSB0) to  usart_nav"
echo "remap the device serial port(ttyUSB1) to  rplidar_nav"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
echo "`rospack find nav_cubot`/script/udev_nav/rplidar_nav.rules"
sudo cp `rospack find nav_cubot`/script/udev_nav/rplidar_nav.rules  /etc/udev/rules.d
sudo cp `rospack find nav_cubot`/script/udev_nav/usart_nav.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
