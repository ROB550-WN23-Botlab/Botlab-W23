#!/bin/sh
echo "Timesync"
pkill timesync

cd /home/pi/botlab-w23/build/bin
./timesync 