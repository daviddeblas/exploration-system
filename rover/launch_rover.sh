#!/bin/bash

roslaunch limo_bringup limo_start.launch &

sleep 10

python3 /inf3995_ws/src/main.py
