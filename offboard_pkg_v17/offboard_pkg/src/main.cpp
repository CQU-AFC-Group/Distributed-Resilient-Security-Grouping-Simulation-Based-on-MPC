#include <ros/ros.h>
#include "offboard_pkg/uav_state_machine.h"
#include <signal.h>

// px4ctrl相关
#include "offboard_pkg/PX4CtrlFSM.h"
#include "offboard_pkg/px4ctrl_utils.h"

#define CONTROL_MODE 1 // 1: 直接MAVROS控制, 2: PX4Ctrl状态机控制

void mySigintHandler(int sig) {
    ROS_INFO("[UAV_Node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_node");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string uav_name;
    pnh.getParam("uav_name", uav_name);

    // 创建 TopologyManager
    std::shared_ptr<TopologyManager> topology_manager = 
        std::make_shared<TopologyManager>(nh);
    
    // 创建状态机
    UAVStateMachine state_machine(nh, uav_name);
    state_machine.setTopologyManager(topology_manager);
    
    #if CONTROL_MODE == 1
    
    ros::Rate rate(100);
    ros::Time last_time = ros::Time::now();
    
    ROS_INFO("[%s] UAV State Machine started", uav_name.c_str());
    
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        
        // 更新状态机
        state_machine.update(dt);
        
        ros::spinOnce();
        rate.sleep();
        last_time = current_time;
    }
    
    #elif CONTROL_MODE == 2
    
    // px4ctrl相关定义
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);
    Px4ctrl_utils utils(nh, fsm, param, uav_name);
    utils.PX4Ctrl_process();

    ros::Rate rate(50);
    ros::Time last_time = ros::Time::now();
    
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        fsm.process(uav_name);
        PX4CtrlFSM::State_t current_state = fsm.get_state();

        if (current_state == PX4CtrlFSM::CMD_CTRL) {
            // 在CMD_CTRL模式下运行状态机
            state_machine.update(dt);
        } else {
            fsm.use_external_control = false;
        }

        ros::spinOnce();
        rate.sleep();
        last_time = current_time;
    }
    
    #endif

    return 0;
}

