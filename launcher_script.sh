#!/bin/bash
/usr/bin/gnome-terminal --tab -e "bash -c 'roslaunch hand_interaction hand_detector.launch  && read'" --tab -e "bash -c 'rosrun RPSProject main && read'"
