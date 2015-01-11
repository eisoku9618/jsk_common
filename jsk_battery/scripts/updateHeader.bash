#!/bin/bash

rm -rf `rospack find jsk_battery`/sketchbook/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py `rospack find jsk_battery`/sketchbook/libraries

sed -i "s#sketchbook.path=.*#sketchbook.path=`rospack find jsk_battery`/sketchbook#g" ~/.arduino/preferences.txt
