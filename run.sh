source devel/setup.bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT
roslaunch lego_loam run.launch&
rosbag play bag/2017-06-08-15-49-45_0.bag --clock --topic /velodyne_points /imu/data&

wait


