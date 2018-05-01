#!/bin/bash

. ../../../../devel/setup.sh

roslaunch kuka_youbot_support gazebo.launch control_type:="Position"
