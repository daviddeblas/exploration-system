#!/bin/bash

cd ./src/ugv_gazebo_sim/limo/limo_gazebo_sim/worlds/
x_y=$(python3 wall_generator.py 3)

x=$(echo $x_y | cut -d ',' -f 1)
y=$(echo $x_y | cut -d ',' -f 2)
yaw=$((RANDOM % 7))
x2=$((((RANDOM % 101) / 10) - 5))
y2=$((((RANDOM % 101) / 10) - 5))
yaw2=$((RANDOM % 7))
roslaunch limo_gazebo_sim limos.launch x:=$x y:=$y yaw:=$yaw x2:=$x2 y2:=$y2 yaw2:=$yaw2 &
