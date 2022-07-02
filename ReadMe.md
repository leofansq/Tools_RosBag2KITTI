## About

2D images and 3D points cloud are the two most commonly used data of intelligent vehicles, which are usually recorded in ROSBAG format via the ROS system in actual engineering practice. However, the visual and LiDAR data in the public dataset (e.g. KITTI) for research are saved as .png and .bin files respectively. To bridge the gap between engineering and research, this repository provides a tool to get .png and .bin from .bag. 

## How to use

### (1) Decode ROSABG

The recorded ROSBAG are firstly decoded into .png for image and .pcd for points cloud.

* Build the project in the ROS workspace
	```bash
	cd catkin_ws
	catkin_make
	```

* [Optional] Setup the parameters in [map_generation_node.cpp](/catkin_ws/src/obstacle_detection/src/map_generation_node.cpp) to achieve the rotation and translation of the coordinate system. 
	> All parameters for the rotation are angular and the clockwise is positive.

* Decode ROSBAG to .png and .pcd, the results are saved in [output](/catkin_ws/output).
	```bash
	# 1st terminal for ROS core
	roscore
	# 2nd terminal for decoding node
	./devel/lib/obstacle_detection/map_generate
	# 3rd terminal for ROSBAG playing, 0.1 means 0.1 times speed
	rosbag play xxx.bag -r 0.1
	```
	> The actual play speed of ROSBAG is determined by the IO performance. Please adjust the speed to ensure the timestamps are within +/- 50 ms.

### (2) Convert .pcd to .bin

The points cloud is further converted from .pcd to .bin.

* Build the project
	```bash
	mkdir CMakeFile
	cd CMakeFile
	cmake ..
	make
	```

* Setup: Move the .pcd files to [pcd](/pcd2bin/pcd), and set the path in [pcd2bin.cpp](/pcd2bin/pcd2bin.cpp).

* Convertion
	```bash
	cd CMakeFile
	./pcd2bin
	```
	> The results are saved in [bin](/pcd2bin/bin).

### (3) Create file list

Generate the list of the subset files for training or validation, e.g. train.txt and val.txt in KITTI.

* Get the file list
	```bash
	cd bin
	ls -1 | grep ".bin$" > list.txt
	python get_list.py
	```
	> The results are saved in [bin](/pcd2bin/bin) named *files_list.txt*.