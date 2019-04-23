// The higher level structure of map otimization is kept the same as LOAM and LeGO-LOAM
#include "LeGOLOAMOpt.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "lego_loam");
    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

    //OdometryAndMapping class
    OdometryAndMapping LeGOLOAM;

    std::thread loopthread(&OdometryAndMapping::loopClosure, &LeGOLOAM);
    std::thread visualizeMapThread(&OdometryAndMapping::visualizeGlobalMap, &LeGOLOAM);

    // Lidar odometry at hgih frequency
    ros::Rate rate(200);
    while (ros::ok()){
        ros::spinOnce();
        LeGOLOAM.run();
        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();
    return 0;
}