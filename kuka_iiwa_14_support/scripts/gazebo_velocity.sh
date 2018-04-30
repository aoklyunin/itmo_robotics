#!/bin/bash

. ../../../../devel/setup.sh

roslaunch kuka_iiwa_14_support gazebo.launch control_type:="Velocity"
