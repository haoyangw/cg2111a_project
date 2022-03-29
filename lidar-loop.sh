#!/bin/bash
LIDAR_PORT="/dev/ttyUSB0"
if [ ! -f "./a.out" ]; then
    exit 1
fi
for (( ; ; ))
do
    echo "Beginning infinite LiDAR scan"
    ./a.out "$LIDAR_PORT"
    sleep 0.5
done
