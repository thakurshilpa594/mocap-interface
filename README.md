## How to run for Amazon Project:

1. Launch Kinova Arm:

`roslaunch kortex_driver kortex_driver.launch ip_address:=192.168.1.10 start_rviz:=false start_moveit:=true cyclic_data_publish_rate:=100 gripper:="robotiq_2f_85"`

2. Run Stretch Sense Glove:

`python src/mocap-interface/scripts/glove_driver.py`

3. Run Backpack ROS Serial (identify the USB port):

`rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0`

4. Launch Haptic Driver:

`roslaunch mocap_interface haptic_feedback.launch`
