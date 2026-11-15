/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TIMEMARKER_H
#define TIMEMARKER_H

#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#ifdef COMPILE_WITH_SIMULATION
#include <ros/ros.h>
#endif

//时间戳  微秒级， 需要#include <sys/time.h> 
inline long long getSystemTime(){
#ifdef COMPILE_WITH_SIMULATION
    // Follow /clock: gait phase must stop when Gazebo physics is paused.
    return static_cast<long long>(ros::Time::now().toNSec() / 1000);
#else
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
#endif
}
//时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond(){
    double time = getSystemTime() * 0.000001;
    return time;
}
//等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(long long startTime, long long waitTime,
                         const bool *running = nullptr){
    if(getSystemTime() - startTime > waitTime){
        // std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        // << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while(getSystemTime() - startTime < waitTime){
#ifdef COMPILE_WITH_SIMULATION
        if (!ros::ok() || (running && !*running)) return;
        // IOROS processes callbacks in this thread; /clock must keep updating
        // while waiting, including when Gazebo is paused or running slowly.
        ros::spinOnce();
        if (getSystemTime() < startTime) return; // Simulation clock reset.
#endif
        usleep(50);
    }
}

#endif //TIMEMARKER_H
