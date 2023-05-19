###
 # @Descripttion: 
 # @Author: Gang Wang
 # @Date: 2023-04-28 18:39:49
### 

# ament cmake
colcon build

# run socket can driver
sleep 1s
{
	gnome-terminal -t "truck_slam" -x bash -c "source install/setup.bash; ros2 launch ros2_socketcan socket_can_receiver.launch.py; exec bash"	
}

# run gnss_ins node driver
sleep 2s
{
	gnome-terminal -t "truck_slam" -x bash -c "source install/setup.bash; ros2 launch gnss_ins_msgs gnss_ins_msg_decode.launch.py; exec bash"
}



# run lidar driver
sleep 2s
{
	gnome-terminal -t "truck_slam" -x bash -c "source install/setup.bash; ros2 launch rslidar_sdk start.py; exec bash"		
}



#run map localization
sleep 3s
{
	gnome-terminal -t "truck_slam" -x bash -c "source install/setup.bash; ros2 launch localization_mapping run_map_localization.launch.py; exec bash"
}
