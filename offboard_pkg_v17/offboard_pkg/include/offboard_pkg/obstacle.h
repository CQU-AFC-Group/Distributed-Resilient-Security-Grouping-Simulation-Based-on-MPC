#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Eigen/Dense>
#include <string>
#include <ros/ros.h>

struct Obstacle {
    std::string name;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double radius;
    bool is_dynamic;
    ros::Time start_time;
};

#endif // OBSTACLE_H
