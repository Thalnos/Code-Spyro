#!/bin/sh
ifconfig wlan0 down
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 essid RobotNetwerk
iwconfig wlan0 channel 10
ifconfig wlan0 192.168.1.2 netmask 255.255.255.0 up
nice -n -20 python /pythonprograms/robotsocketsplinenewRLS.py &

