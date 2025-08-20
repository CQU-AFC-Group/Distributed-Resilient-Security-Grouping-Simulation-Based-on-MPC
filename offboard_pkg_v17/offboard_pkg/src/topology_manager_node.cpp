#include <ros/ros.h>
#include "offboard_pkg/topology_manager.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "topology_manager_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting independent Topology Manager Node...");
    
    TopologyManager topology_manager(nh);
    
    ros::spin();
    
    return 0;
}
