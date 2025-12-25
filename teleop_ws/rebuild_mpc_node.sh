#!/bin/bash


colcon build --packages-select mpc_interpolation --cmake-clean-cache


sudo setcap cap_sys_nice=eip install/mpc_interpolation/lib/mpc_interpolation/mpc_interpolation_node



# gnome-terminal -- bash -c "ros2 run rqt_plot rqt_plot; exec bash"


# ros2 run mpc_interpolation mpc_interpolation_node
