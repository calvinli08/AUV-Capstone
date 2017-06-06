#!/bin/bash

#Check that all mission necessary components are ready, and start a mission if they are
#Checks:
#1) Check for the presence of the modified mavproxy_*.py files and the mavproxy_auv.py file
#2) Check for waypoints.txt file
#3) Check for MAVProxy

echo "This script prepares all mission parameters and launches MAVProxy. As such, this script will not exit unless MAVProxy is exited. "

#1) Update and check for the presence of the modified mavproxy_*.py files and the mavproxy_auv.py file
echo "Updating git repo and copying over files"
/bin/bash /home/pi/update_custom_mavproxy_files.sh

$b = head -2 /usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_battery.py
if [ "$b" = "'''Custom'''" ]; then
  echo "Custom battery module exists";
else
  echo "Custom battery module does not exist";
  exit 1
fi

#2) Check for waypoints.txt file
if [ -rwx /home/pi/waypoints.txt ]; then
  echo "Waypoints file exists";
else
  echo "Waypoints file does not exist";
  exit 2
fi

#3) Check for MAVProxy
if [ -d "/usr/local/lib/python2.7/dist-packages/MAVProxy" ]; then
  echo "MAVProxy installed";
else
  echo "MAVProxy is not installed";
  exit 3
fi


#If all checks pass, start the mission

#Start MAVProxy last or else it will capsture all other input
#copied from start_mavproxy_telem_splitter.sh
mavproxy.py --master=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00,11520
            --source-system=200 \
            --cmd="set heartbeat 0" \
            --out udpin:localhost:9000 \
            --out udpbcast:172.24.1.255:14550
            --load-module=wp,dataflash_logger,battery, serial, auv

exit 0
