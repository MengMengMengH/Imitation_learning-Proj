export ROS_DOMAIN_ID=211
source ~/Imitation_learning-Proj/teleop_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

