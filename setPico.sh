sudo picotool reboot
pkill timesync
pkill pico_shim
pkill motion_controller
pkill drive_square
cd /home/pi/botlab-w23/build/bin
./pico_shim & ./timesync & ./motion_controller & ./drive_square