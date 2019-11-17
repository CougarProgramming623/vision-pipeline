#!/bin/bash
if [[ $# -ne 2 ]]; then
    echo "Usage: ./calibrate.sh {points} {pictures}"
    echo "Use at least 15 pictures"
    exit 1
fi
./callibration -w=$1 -h=$1 -s=1.5 -o=param.yaml -n=$2
