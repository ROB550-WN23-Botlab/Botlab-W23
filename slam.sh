#!/bin/sh
echo "Running SLAM" 

bash stop.sh

gnome-terminal -- ./ts.sh
sleep 2s 
gnome-terminal -- ./ps.sh
sleep 2s
gnome-terminal -- ./ld.sh
sleep 2s
gnome-terminal -- ./bg.sh