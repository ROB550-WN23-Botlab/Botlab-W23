#!/bin/sh
echo "Lidar"

pkill rplidar_driver
cd ~/botlab-w23/system_compilation/bin
./rplidar_driver    
