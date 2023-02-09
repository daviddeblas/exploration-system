#!/bin/sh
# Lancer roscore en arrière plan
roscore &
# Attendre que roscore démarre
sleep 5
# Lancer le script python
python3 main.py
