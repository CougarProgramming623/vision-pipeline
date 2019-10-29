#!/bin/bash
mjpg_streamer -i "input_uvc.so -d /dev/video0 $OPTIONS" -o "output_http.so -p 7777 -w /usr/share/mjpg-streamer/www"
