# connect IMUs
1. run `roscore` first  
2. connect LPMS-B2 thorugh bluetooth: ``rosrun openzen_sensor openzen_sensor_node __name:="imu3" _sensor_interface:="Bluetooth" _sensor_name:="00:04:3E:9B:A3:8F" imu:=/imu3``  
connect other IMUs through cable: ``rosrun openzen_sensor openzen_sensor_node __name:="imu3" _sensor_name:="devicefile:/dev/ttyUSB0" _frame_id:="imu" imu:=/imu3``

3. a shotcut way:  
    * check if screen.rc already exists in qolo_ros/run_scripts,   
    * if yes, run ``screen -S imu_conneciton -c /home/qolo/catkin_ws/src/qolo_ros/qolo_package/src/screen.rc`` or run ``connnect_imus`` as an alias, (assuming already put ``alias connect_imus="screen -S imu_conneciton -c /home/qolo/catkin_ws/src/qolo_ros/run_scripts/screen.rc`` into ~/.bashrc  


# ROSLAUNCH initiation 
1. run roslaunch IMU_ADQ_pck launch_urdf_IMU.launch

2. open rqt (run rqt in a second terminal)

3. Add the pluggin "Easy message publisher"

4. Open the topics "/adq/reference/start" (for starting the motion sequence of the reference) and "/adq/imu_rel/offset" (for adding an offset to correct the marker position) 