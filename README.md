# Final-Project
This is Team 11's final project git repository for EECS 568: Mobile Robotics. The title of our project is **Implementing LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping**. Team members include: Bowen Mu, Chien Erh Lin, Haochen Wu, and Weilin Xu.

<a href="https://imgflip.com/gif/2zazp8"><img src="https://i.imgflip.com/2zazp8.gif" title="made at imgflip.com"/></a>

## Abstract
LeGO-LOAM, a lightweight and ground-optimized lidar odometry and mapping method, is able to do six degree-of-freedom pose estimation for real time with ground vehicles \cite{legoloam2018}. This project re-implemented LeGO-LOAM which reduced computational expense while keeping similar accuracy compared to LOAM method. The whole LeGO-LOAM system is implemented in Robot Operating System (ROS). The data used for simulation is raw data (with Velodyne LiDAR data and IMU) from KITTI odometry benchmark dataset. Sequences 00, 05, and 08 are tested in our project. The mapping and odometry results of these sequences are presented in this report. Our code is public in [this GitHub Repository](https://github.com/mobileteam11/Final-Project). Video of implementing LeGO-LOAM on Kitti sequence 00 and sequence 08 are available at [Kitti sequence 00 youtube link](https://youtu.be/z3yXCRAMC18) and [Kitti sequence 08 youtube link](https://youtu.be/nJ74Uk6m10U).
## Prerequisites
- First, install [ROS](http://wiki.ros.org/ROS/Installation). We used ROS-kinetic on Ubuntu 16.04. Please follow the instruction of ROS installation page.
- [gtsam](https://github.com/borglab/gtsam). GTSAM is a library of C++ classes that implement smoothing and mapping (SAM). You can install it by following code.
```
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam/
mkdir build
cd build
cmake ..
make check 
make install
```

## Installing and Compile
To install and compile our code, please clone this respository in src/ under catkin worksspace.
```
mkdir -p ./loam_ws/src/
cd loam_ws/src/
git clone https://github.com/mobileteam11/Final-Project.git
mv Final-Project/ LeGO-LOAM/
cd ..
```
in /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake
change :17 `find_dependency` to `find_package`
```
catkin_make -j1
```

## Visualizing Odometry and Point Cloud Map
After building up the code, you can download a sample ros bag file from [this link](https://drive.google.com/open?id=1sxApV5dmFf6UF1WJs1LW3-WQaRopNyDq) and put it in Final-Project/bag/ folder. Right under the catkin workspace, simply run the code below in command line will run rviz and see the visualized result.
```
./src/LeGO-LOAM/run.sh
```
or command in different terminal. Change topic names from `/velodyne_points` and `/imu/data` to the topic name of the rosbag.
```
source ./devel/setup.bash
roslaunch lego_loam run.launch
rosbag play PATH_TO_BAG_FILE.bag --clock --topic /velodyne_points /imu/data
```

## Kitti Odometry Dataset to rosbag
Kitti dataset are provided to allow the usage of the laser point clouds, gps data, the right
camera image and the grayscale images. It provides the mapping of the training set to the raw data of the KITTI dataset.

To run kitti data in our program, [kitti2bag](https://github.com/tomas789/kitti2bag) is used to transform kitti raw data to rosbag. First, download raw sequences **synced+rectified data** and **calibration** data from [The KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/raw_data.php). 


The sequences that provide ground truth are shown in the below table. The [odometry eval kit](http://kitti.is.tue.mpg.de/kitti/devkit_odometry.zip) includes description.

The following table lists the name, start and end frame of each sequence that
has been used to extract the visual odometry / SLAM training set

| Nr. |Sequence name |Start | End |
| --- | --- | --- | --- |
| 00 | 2011_10_03_drive_0027 | 000000 | 004540 |
| 01 | 2011_10_03_drive_0042 | 000000 | 001100 |
| 02 | 2011_10_03_drive_0034 | 000000 | 004660 |
| 03 | 2011_09_26_drive_0067 | 000000 | 000800 |
| 04 | 2011_09_30_drive_0016 | 000000 | 000270 |
| 05 | 2011_09_30_drive_0018 | 000000 | 002760 |
| 06 | 2011_09_30_drive_0020 | 000000 | 001100 |
| 07 | 2011_09_30_drive_0027 | 000000 | 001100 |
| 08 | 2011_09_30_drive_0028 | 001100 | 005170 |
| 09 | 2011_09_30_drive_0033 | 000000 | 001590 |
| 10 | 2011_09_30_drive_0034 | 000000 | 001200 |

## Result
Our result of running Kitti Sequences are shown below. We compared between simulation result and groundtruth. In addition, the mapping result for Sequence 00, 05, and 08 of Kitti are shown below.
![seq00](/result/00_cmp.png)
![seq00](/result/00.png)
![seq00](/result/00_error.png)
![seq05](/result/05_cmp.png)
![seq05](/result/05.png)
![seq05](/result/05_error.png)
![seq08](/result/08_cmp.png)
![seq08](/result/08.png)
![seq08](/result/08_error.png)
