#!/bin/bash

v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=9 
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=30

# frame rate
v4l2-ctl -d /dev/video0 --set-parm 20
