#!/bin/sh
echo "Assuming you already ran the 'make' command" 
echo "Make sure you are running from Pi directly"
echo "Running only action mode - To change, edit t3.sh file"

gnome-terminal -- ./t3.sh
sleep 5s 
gnome-terminal -- ./bg.sh
sleep 5s
gnome-terminal -- ./t2.sh