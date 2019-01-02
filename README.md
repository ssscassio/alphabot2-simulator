# alphabot2-simulator
# Launch World
roslaunch alphabot2_world spawn_world.launch

# Launch Robot
roslaunch alphabot2_world spawn_robot.launch

# Launch Camera
roslaunch alphabot2_tracking run.launch

rosrun image_view image_view image:=/alphabot2_camera/image_raw

# Run Control
chmod +x src/alphabot2-simulator/alphabot2_control/src/alphabot2_control_node.py

rosrun alphabot2_control alphabot2_control_node.py

rostopic pub /alphabot2/cmd_vel geometry_msgs/Twist (TAB)
rostopic echo /cmd_vel (ENTER)

# Launch Pantilt
roslaunch alphabot2_pantilt_control alphabot2_pantilt_control.launch

rostopic pub -1 /alphabot2/joint_lower_camera_position_controller/command std_msgs/Float64 "data: 0.78"