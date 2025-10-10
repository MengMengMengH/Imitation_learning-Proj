### Before connect to rokae robot ,execute:

    sudo ip addr add 192.168.0.100/24 dev enp5s0(Your internet port)


### After colcon build --package-select mpc_interpolation, execute:

#### Once ####
1. `sudo nano /etc/ld.so.conf.d/ros2-humble.conf`
2. add  /opt/ros/humble/lib
        /home/{user}/Imitation_learning-Proj/teleop_ws/install/mpc_interpolation/lib
3. `sudo ldconfig`
4. Checkout `ldconfig -p | grep librclcpp`

#### When Rebuild
    ~/Imitation_learning-Proj/teleop_ws:sudo setcap cap_sys_nice=eip install/mpc_interpolation/lib/mpc_interpolation/mpc_interpolation_node
Then mpc_interpolation node could run.