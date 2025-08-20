// uav_agent.h
#ifndef UAV_AGENT_H
#define UAV_AGENT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <geometry_msgs/TwistStamped.h>
#include "offboard_pkg/custom_trajectory_planner.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <nlopt.hpp>//求解器
#include <casadi/casadi.hpp>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include "MPCController.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "offboard_pkg/NonlinearESO.hpp"

class UAVAgent {
public:
    UAVAgent(const ros::NodeHandle& nh, const std::string& uav_name, double offset_x, double offset_y, double local_offset_x_, double local_offset_y_);
    // 基础函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    bool armAndSetMode();
    void publishSetVelocity();
    void updateTrajectory(double dt);
    // 自定义轨迹规划：
    void setTrajectoryPlanner(CustomTrajectoryPlanner* planner);
    // 分布式
    void publishAgentState();
    void publishOffset();
    void neighborCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& neighbor_name);
    void registerNeighbor(const std::string& neighbor_name);
    void neighborPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& neighbor_name);
    void neighborOffsetCallback(const geometry_msgs::Point::ConstPtr& msg, const std::string& neighbor_name);
    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void loadObstaclesFromParamServer();
    void neighborTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& neighbor_name);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    Eigen::Vector3d calculateObstacleAvoidanceForce();
    ros::Publisher vel_pub_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped pose_;
    ros::Publisher local_pos_pub_;
    // 轨迹规划
    // 轨迹规划器
    CustomTrajectoryPlanner* planner_;
    bool reached_goal;
    double traj_start_time_;
    
    // UAV0的参考偏移
    double offset_x0 = 2.0;
    double offset_y0 = 0.0;
    // 固定的本地坐标系偏移量（初始化后不再改变）
    double local_offset_x_;
    double local_offset_y_;
    // 避障斥力增益
    double K_avoidance = 1.0;

    // 获取障碍物列表
    std::vector<Obstacle> getObstacles() const;
    // 障碍物列表
    std::vector<Obstacle> obstacles_;
    void resetTrajectoryTime();
    bool detectObstacleAhead(double look_ahead_distance, double safety_radius);
    void publishObstacles();
    // 避障相关
    bool in_avoidance_mode_ = false;
    CustomTrajectoryPlanner* original_planner_ = nullptr;
    double original_traj_start_time_ = 0;
    bool safety_timer_started_ = false;
    double safety_start_time_ = 0;
    // 用于平滑过渡的变量
    bool transitioning_to_original_ = false;
    double transition_start_time_ = 0.0;
    double transition_duration_ = 2.0; // 过渡时间，单位：秒
    double transition_target_time_ = 0.0; // 目标时间点
    Eigen::Vector3d transition_start_pos_;
    Eigen::Vector3d transition_start_vel_;
    
    // MPC参数
    struct MPCParams {
        // 控制权重
        double w_trajectory = 1.0;   // 轨迹跟踪权重
        double w_obstacle = 2.0;     // 障碍物避让权重
        double w_velocity = 0.5;     // 速度平滑权重
        
        // 安全距离
        double safe_distance = 0.8;  // 障碍物安全距离
        
        // 控制限制
        double max_velocity = 2.0;   // 最大速度限制
    };
    MPCParams mpc_params_;

    // ===== 定义优化数据结构 =====
    struct OptimizationData {
        // 当前状态
        Eigen::Vector2d pos_curr_xy;
        Eigen::Vector2d vel_curr_xy;
        Eigen::Vector2d desired_pos_xy;
        Eigen::Vector2d desired_vel_xy;
        
        // 邻居信息
        std::vector<Eigen::Vector2d> neighbor_positions_xy;
        std::vector<Eigen::Vector2d> neighbor_velocities_xy;
        
        // 障碍物信息
        std::vector<Obstacle> obstacle_poses;
        
        // 参数
        int P;
        double dt;
        double w_traj, w_form, w_obs, w_input, w_smooth;
        double d_formation, d_safe_obs;
        
        // 成本存储
        double* traj_cost_final;
        double* form_cost_final;
        double* obs_cost_final;
        double* input_cost_final;
        double* smooth_cost_final;
        double* total_cost_final;
        
        std::string uav_name;

        // 新增：约束标识参数
        int constraint_k;      // 当前约束的时间步索引
        int constraint_type;   // 约束类型
        double max_accel;      // 最大加速度

        std::vector<Eigen::Vector2d> desired_positions_xy;  // 添加精确位置序列
        std::vector<Eigen::Vector2d> desired_velocities_xy; // 添加精确速度序列
    };OptimizationData opt_data;
    

    // 约束类型枚举
    enum ConstraintType {
        DYNAMICS_X = 0,   // x方向动力学约束
        DYNAMICS_Y = 1    // y方向动力学约束
    };

    
    // 新增动力学优化数据结构定义（需要添加到头文件中）
    struct DynamicsOptData {
        Eigen::Vector3d current_pos;
        Eigen::Vector3d current_vel;
        std::vector<Eigen::Vector3d> desired_positions;
        std::vector<Eigen::Vector3d> desired_velocities;
        std::vector<Eigen::Vector3d> neighbor_positions;
        std::vector<Eigen::Vector3d> neighbor_velocities;
        std::vector<Obstacle> obstacle_poses;
        
        int P;
        double dt;
        double mass;
        double gravity;
        double thr2acc;
        
        double w_pos, w_vel, w_thrust, w_angle, w_smooth;
        
        double* traj_cost_final;
        double* form_cost_final;
        double* control_cost_final;
        double* smooth_cost_final;
        double* total_cost_final;
        
        std::string uav_name;
    };
    geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw);
    // 修改后的MPC控制结构体，用于返回姿态和推力信息
    struct MPCControlOutput {
        geometry_msgs::Quaternion attitude;  // 期望姿态四元数
        double thrust;                       // 归一化推力 [0,1]
        bool success;                        // 优化是否成功
    };
    // MPC 控制器
    MPCControlOutput calculateMPCControl(const Eigen::Vector3d& desired_pos, 
        const Eigen::Vector3d& desired_vel, 
        double current_time,
        const std::vector<Eigen::Vector3d>& neighbor_positions,
        const std::vector<Eigen::Vector3d>& neighbor_velocities);

    ros::Publisher attitude_cmd_pub_;        // 姿态指令发布器
    ros::Publisher attitude_target_pub_;     // mavros姿态目标发布器
    ros::ServiceClient arming_client_;

    // 状态获取
    double getThr2AccFromPX4();
    ros::ServiceClient param_get_client;
    void debugMPCSolution(const casadi::DM& U_opt, 
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_vel,
        const Eigen::Vector3d& desired_pos,
        double thr2acc) ;

    // MPC控制器 - 需要显式初始化
    mutable std::unique_ptr<MPCController> mpc_controller_;

     // 获取邻居状态信息
    std::vector<std::string> neighbor_names_;
    std::vector<Eigen::Vector3d> getNeighborPositions() const;
    std::vector<Eigen::Vector3d> getNeighborVelocities() const;
    std::vector<std::string> getNeighborNames() const { return neighbor_names_; }

    void updateOffset(double new_offset_x, double new_offset_y);

    NonlinearESO observer;
 

private:
    std::string uav_name_;
    double offset_x_, offset_y_;
    double x_position_ = 0.0;
    double velocity_ = 0.1;

    ros::Time last_request_;
    ros::NodeHandle nh_ns_;
    ros::NodeHandle nh_obs;

    ros::Subscriber state_sub_;
    ros::Publisher offset_pub_;
    ros::Publisher error_pub_;
    // 在控制器类的构造函数或初始化函数中
    ros::Publisher desired_pos_pub_;
    ros::Publisher desired_path_pub_;
    ros::Publisher agent_path_pub_;
    ros::Subscriber local_pose_sub_;
    geometry_msgs::TwistStamped vel_msg_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    nav_msgs::Path desired_path;
    nav_msgs::Path agent_path;
    ros::Publisher obstacle_marker_pub_;

    ros::Publisher agent_state_pub_;
    std::map<std::string, ros::Subscriber> neighbor_subs_;
    std::map<std::string, geometry_msgs::PoseStamped> neighbor_poses_;
    std::map<std::string, geometry_msgs::Point> neighbor_offsets_;
    std::map<std::string, ros::Subscriber> neighbor_offset_subs_;
    std::unordered_map<std::string, geometry_msgs::TwistStamped> neighbor_twists_;
    std::map<std::string, ros::Subscriber> neighbor_twist_subs_;
    geometry_msgs::TwistStamped current_velocity_; // 当前实际速度
    ros::Subscriber velocity_sub_;  // 速度订阅

    std::vector<Obstacle> obstacle_poses_;
    Eigen::Vector3d final_pos;
    Eigen::Vector3d final_vel = Eigen::Vector3d::Zero();
    
};

#endif  // UAV_AGENT_H
