#!/bin/bash
sleep 10

source /home/beachbot/.bashrc


# Check if external monitor connected and turned on:
export DISPLAY=:0.0
connected=$(su beachbot -c 'DISPLAY=:0.0; xrandr' | awk '/ connected/ {count++} END {print count}')
echo "monitor status is $connected"
if [ "$connected" = "1" ]; then
echo "Monitor is connected!"
# Tasks todo when a monitor is connected (we are in OIST):
# 1. connect to oist WIFI (turn off hotspot and turn on again -> then trick to click on "accept")
# 2. update time from server!
else
echo "Monitor not conected -> Robot is in the wild!"
# Tasks todo:
# 1. start robot server
# 
sudo nmcli d wifi hotspot ifname wlan0 ssid beachbot password beachbot
echo "start server..."
echo $PWD
echo $USER
#rm -f nohup.out
#su beachbot -c 'nohup python /home/beachbot/src/python-interface/app/nicegui_controller.py &'
#su beachbot -c 'python /home/beachbot/src/python-interface/app/nicegui_controller.py' &
#python /home/beachbot/src/python-interface/app/nicegui_controller.py
su beachbot -c 'python /home/beachbot/src/python-interface/app/nicegui_controller.py'
echo "done?!"
fi


