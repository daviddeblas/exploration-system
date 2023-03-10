#!/bin/bash
cd /inf3995_ws/
source devel/setup.bash
cd ./src/ugv_gazebo_sim/limo/limo_gazebo_sim/worlds/
x_y=$(python3 wall_generator.py 3)
x=$(echo $x_y | cut -d ',' -f 1)
y=$(echo $x_y | cut -d ',' -f 2)
yaw=$((RANDOM % 7))
x2=$((((RANDOM % 101) / 10) - 5))
y2=$((((RANDOM % 101) / 10) - 5))
yaw2=$((RANDOM % 7))

roslaunch limo_gazebo_sim limos.launch x:=$x y:=$y yaw:=$yaw x2:=$x2 y2:=$y2 yaw2:=$yaw2 &
sleep 15s
(
    source /catkin_ws/devel_isolated/setup.bash
    roslaunch cartographer_ros carto.launch &
)
(
    python3 /inf3995_ws/src/main_robot1.py
)
