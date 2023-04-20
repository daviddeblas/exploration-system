#!/bin/bash
cd /inf3995_ws/src/ugv_gazebo_sim/limo/limo_gazebo_sim/worlds/
python3 wall_generator.py 3

x=$((((RANDOM % 101) / 10) - 5)) # Les positions possibles sont entre -5 et 5
y=$((((RANDOM % 101) / 10) - 5)) # Les positions possibles sont entre -5 et 5
yaw=$((RANDOM % 7)) # Environ 2pi pour l'orientation

roslaunch limo_gazebo_sim robots.launch x:=$x y:=$y yaw:=$yaw &
sleep 20

if [ "$USE_ONLY" != "COGNIFLY" ]; then
python3 /inf3995_ws/src/main_robot1.py &
fi
if [ "$USE_ONLY" != "LIMO" ]; then
python3 /inf3995_ws/src/main_robot2.py &
python3 /inf3995_ws/src/simple_quad/src/simple_quad/show_image.py &
fi
