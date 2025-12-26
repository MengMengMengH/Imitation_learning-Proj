#!/bin/bash

cd ~/Imitation_learning-Proj/teleop_ws/

rm -rf ./build ./install ./log

colcon build --packages-ignore mpc_interpolation arm_tele_mujo start_up

colcon build --packages-select arm_tele_mujo start_up --symlink-install

colcon build --packages-select mpc_interpolation --cmake-clean-cache

sudo setcap cap_sys_nice=eip install/mpc_interpolation/lib/mpc_interpolation/mpc_interpolation_node
