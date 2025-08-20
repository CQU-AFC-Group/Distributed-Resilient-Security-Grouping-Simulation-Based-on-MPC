#ifndef PX4CTRL_UTILS_H
#define PX4CTRL_UTILS_H

#include <ros/ros.h>
#include "offboard_pkg/PX4CtrlFSM.h"        // 相对路径
#include <signal.h>

// 假设这些数据结构在 px4ctrl 已有定义（如 State_Data_t 等）
class PX4CtrlFSM; // 前向声明，避免头文件依赖

class Px4ctrl_utils
{
    private:
        ros::NodeHandle nh_ns_;
        ros::NodeHandle nh_rc;
        PX4CtrlFSM& fsm_;
        const Parameter_t& param_;
        std::string uav_name_;

        ros::Subscriber state_sub;
        ros::Subscriber extended_state_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber cmd_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber rc_sub;
        ros::Subscriber bat_sub;
        ros::Subscriber takeoff_land_sub;

    public:
        
        Px4ctrl_utils(const ros::NodeHandle& nh, PX4CtrlFSM& fsm, const Parameter_t& param, const std::string uav_name);
        void PX4Ctrl_process();

};
    // 封装初始化与订阅发布逻辑的函数




#endif