##########
# move the robot in rviz
roslaunch jbot_bringup fake_jbot.launch
rviz
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- '[0.1,0,0]' '[0,0,-0.5]'

##########
# sim_move_in_moveit
roslaunch jbot_moveit_config demo.launch sim_diff_move:=true
#then choose the 'Global Options'->'Fixed Frame' to 'odom', will see the robot move in rviz, and also can plan the arms on 'MotionPlanning' panel.

##########
# rcv car base velocity and transform to odomtry msg
roslaunch jbot_moveit_config demo.launch
rosrun jbot_control jbot_base_driver.py
#witch will rcv the `0.1|0|0~ format string from serial(such as serial assitant), then choose from rviz of the 'Global Options'->'Fixed Frame' to 'odom', will see the robot move in rviz.

##########
# ps3 joy to control robot
sudo bash
rosrun ps3joy sixpair
rosrun ps3joy ps3joy.py  #ready to active joy, if screen shows 'Connection activated', then next

rosrun joy joy_node # pub cmd_vel
# or
roslaunch jbot_control ps3_control.launch  # pub joy topic

##########
# show lidar data in rviz
roscd rplidar_ros/launch
#change serial_port to /dev/ttyUSB_rplidar
roslaunch rplidar_ros view_rplidar.launch 

##########
## just test double arm move
rosrun jbot_arm_driver jbot_arms_controller.py
# test move and run on rviz
roslaunch jbot_arm_driver jbot_arms_test_move.launch
## test arm planning
roslaunch jbot_arm_driver jbot_arms_planning.launch
## test ps3 to control jbot_arms
rosrun ps3joy ps3joy.py  #ready to active joy, if screen shows 'Connection activated', then next
rosrun joy joy_node
rosrun jbot_arm_driver jbot_arms_driver.py

##########
## test mapping
rosrun ps3joy ps3joy.py
roslaunch jbot_control ps3_control.launch
roslaunch jbot_nav gmapping_demo.launch

## test navigation
roslaunch jbot_control ps3_control.launch use_joy:=false
roslaunch jbot_nav gmapping_demo.launch
# if control with keyboard: 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py , # z to down the speed of the vel to 0.2

# save map
rosrun mapserver mapsaver -f my_map

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "base_footprint"},pose:{position:{x: 1.0,y: 0,z: 0}, orientation:{x: 0,y: 0,z: 0,w: 1}}}'

## car calibration
roslaunch jbot_nav car_calibration.launch
#nest steps in car_calibration.launch

##fake mapping
roscore
roslaunch jbot_nav fake_gmapping_demo.launch sim_time:=true
roscd jbot_nav/bag_files
rosbag play my_bag3.bag
#rviz show the map.

##fake nav
roslaunch jbot_bringup fake_jbot.launch
roslaunch jbot_nav nav_with_map.launch sim:=true
rviz
roslaunch rbx1_nav move_base_square.py

##real nav
roslaunch jbot_nav nav_with_map.launch
rviz 
# give a target position or roslaunch rbx1_nav move_base_square.py

##amcl

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "base_link" }, pose: { position: { x: 1.0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'  


#bind usb seriral port,
#udevadm info --name=ttyUSB0 --attribute-walk
sudo gedit /etc/udev/rules.d/20-usb-serial.rules
KERNEL=="ttyUSB*"  MODE="0777"
KERNEL=="ttyS*"  MODE="0777"
KERNELS=="1-1.2.4", SUBSYSTEM=="tty", SYMLINK+="ttyUSB_rplidar"
KERNELS=="1-1.1.2", SUBSYSTEM=="tty", SYMLINK+="ttyUSB_left_arm"
KERNELS=="1-1.1.1", SUBSYSTEM=="tty", SYMLINK+="ttyUSB_right_arm"
KERNELS=="1-1.1.4", SUBSYSTEM=="tty", SYMLINK+="ttyUSB_base_control"
sudo udevadm trigger



