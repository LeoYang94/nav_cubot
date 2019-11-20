#!/bin/bash

echo "delete remap the device serial port(ttyUSB1) to  rplidar_nav"
echo "delete remap the device serial port(ttyUSB0) to  usart_nav"
echo "sudo rm   /etc/udev/rules.d/rplidar_nav.rules"
echo "sudo rm   /etc/udev/rules.d/usart_nav.rules"
sudo rm   /etc/udev/rules.d/rplidar_nav.rules
sudo rm   /etc/udev/rules.d/usart_nav.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
