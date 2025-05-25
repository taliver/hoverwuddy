#!/bin/bash
date >> /tmp/starting
/usr/bin/python3 /home/pi/hoverwuddy/fullcontrol.py /dev/ttyS0
date >> /tmp/stopping


