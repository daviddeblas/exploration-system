#!/bin/bash
x=$((((RANDOM % 101) / 10) - 5))   # Les positions possibles sont entre -5 et 5
y=$((((RANDOM % 101) / 10) - 5))   # Les positions possibles sont entre -10 et 10
yaw=$((RANDOM % 7))         # Environ 2pi pour l'orientation
x2=$((((RANDOM % 101) / 10) - 5))
y2=$((((RANDOM % 101) / 10) - 5))
yaw2=$((RANDOM % 7))
roslaunch limo_gazebo_sim limos.launch x:=$x y:=$y yaw:=$yaw x2:=$x2 y2:=$y2 yaw2:=$yaw2 &
