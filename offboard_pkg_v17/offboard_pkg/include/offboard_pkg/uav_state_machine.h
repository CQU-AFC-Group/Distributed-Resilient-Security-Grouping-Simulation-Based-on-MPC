#ifndef UAV_STATE_MACHINE_H
#define UAV_STATE_MACHINE_H

#include <ros/ros.h>
#include <string>
#include <memory>
#include "offboard_pkg/uav_agent.h"
#include "offboard_pkg/custom_trajectory_planner.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include "offboard_pkg/formation_manager.h"
#include "offboard_pkg/topology_manager.h" 
#include <std_msgs/Time.h>

class TopologyManager;

// 状态机状态枚举
enum class UAVState {
    IDLE,              // 空闲状态
    INITIALIZING,      // 初始化状态
    PRE_PLANNING,      // 预规划状态
    PLANNING,          // 轨迹规划状态
    TAKING_OFF,        // 起飞状态
    HOVERING,          // 悬停状态
    EXECUTING,         // 执行轨迹状态
    FORMATION_CHANGING, // 队形变换状态
    EMERGENCY,         // 紧急状态
    LANDING,            // 降落状态
};

// 状态机事件枚举
enum class UAVEvent {
    START_MISSION,
    OBSTACLES_LOADED,
    WAYPOINTS_VALIDATED,  // 起点终点验证完成
    PRE_PLANNING_COMPLETE, // 预规划完成
    PLANNING_COMPLETE,
    PLANNING_FAILED,
    TAKEOFF_COMPLETE,
    START_EXECUTION,
    GOAL_REACHED,
    NEW_GOAL_RECEIVED,
    EMERGENCY_STOP,
    RESET_REQUEST,
    FORMATION_CHANGE_REQUEST,  // 队形变换请求
    FORMATION_CHANGE_COMPLETE  // 队形变换完成
};

class UAVStateMachine {
public:
    UAVStateMachine(ros::NodeHandle& nh, const std::string& uav_name);
    ~UAVStateMachine();
    
    // 主要接口
    void update(double dt);
    void handleEvent(UAVEvent event);
    UAVState getCurrentState() const { return current_state_; }
    // 设置 TopologyManager 的接口
    void setTopologyManager(std::shared_ptr<TopologyManager> topology_manager) {
        topology_manager_ = topology_manager;
    }
    
    // ROS回调函数
    void terminalTakeoffCallback(const std_msgs::Bool::ConstPtr& msg);
    void startControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void newGoalCallback(const geometry_msgs::Point::ConstPtr& msg);
    void emergencyCallback(const std_msgs::Bool::ConstPtr& msg);
    void resetCallback(const std_msgs::Bool::ConstPtr& msg);
    void missionCallback(const std_msgs::Bool::ConstPtr& msg);
    void formationChangeCallback(const std_msgs::String::ConstPtr& msg);
    void handlePrePlanningState(double dt);
    // 邻居状态回调和处理函数
    void neighborStateCallback(const std_msgs::String::ConstPtr& msg, const std::string& neighbor_name);
    void reportNeighborStates();
    void neighborReportTimerCallback(const ros::TimerEvent& event);
    std::string stateToString(UAVState state);

private:
    // 状态处理函数
    void handleIdleState(double dt);
    void handleInitializingState(double dt);
    void handleTakingOffState(double dt);
    void handleHoveringState(double dt);
    void handleExecutingState(double dt);
    void handleEmergencyState(double dt);
    void handleLandingState(double dt);
    
    // 状态转换函数
    void transitionTo(UAVState new_state);
    void onEnterState(UAVState state);
    void onExitState(UAVState state);
    
    // 辅助函数
    bool loadObstaclesAndWaypoints();
    bool planTrajectory();
    bool isGoalReached();
    void publishStateInfo();
    void resetTrajectory();

    // 队形变换相关函数与变量
    bool formation_change_requested_;
    std::string target_formation_type_;
    ros::Time formation_change_start_time_;
    bool in_formation_transition_;

    // 队形变换
    void checkAndHandleFormationChange();
    bool detectTopologyChange();
    void setupHoveringFormationTransition();
    void setupExecutingFormationTransition(double current_time);
    Eigen::Vector3d calculateReferenceFormationCenter(double current_time);
    int current_online_count, last_known_online_count_;
    ros::Time last_topology_change_time_;
    
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Publisher state_pub_;
    ros::Publisher status_pub_;
    ros::Subscriber terminal_takeoff_sub_;
    ros::Subscriber distuibuted_control_sub_;
    ros::Subscriber mission_start_sub_;
    ros::Subscriber formation_change_sub_;
    ros::Subscriber new_goal_sub_;
    ros::Subscriber emergency_sub_;
    ros::Subscriber reset_sub_;
    ros::Subscriber leader_path_sub_;
    ros::Subscriber takeoff_land_sub_;
    ros::Publisher leader_path_pub_;
    ros::Subscriber individual_mission_start_sub_;
    ros::Subscriber individual_terminal_takeoff_sub_;
    ros::Subscriber individual_emergency_sub_;
    
    // 状态机变量
    UAVState current_state_;
    UAVState previous_state_;
    ros::Time state_enter_time_;
    ros::Time last_obstacle_check_;
    
    // UAV相关
    std::unique_ptr<UAVAgent> agent_;
    std::unique_ptr<CustomTrajectoryPlanner> planner_;
    std::string uav_name_;
    
    // 目标和路径
    Eigen::Vector3d current_goal_;
    Eigen::Vector3d start_position_;
    nav_msgs::Path::ConstPtr leader_path_msg_;
    bool received_leader_path_;
    Eigen::Vector3d hover_target_position_;  // 固定的悬停目标位置
    
    // 参数
    double offset_x_, offset_y_;
    std::vector<std::string> neighbors_;
    double goal_tolerance_;
    double obstacle_check_interval_;
    double takeoff_height_;
    double t_reach;
    std::map<std::string, Eigen::Vector3d> formation_offsets;
    double pre_offset_x, pre_offset_y;
    Eigen::Vector3d temp_current_goal_;
    double local_offset_x, local_offset_y;
    std::map<std::string, Eigen::Vector3d> formation_centers_;  // 存储各邻居的编队中心
    std::map<std::string, ros::Subscriber> formation_center_subs_;  // 编队中心订阅者
    ros::Publisher formation_center_pub_;
    Eigen::Vector3d shared_formation_center_;
    Eigen::Vector3d recovery_pos;

    // 状态标志
    bool mission_started_;
    bool distributed_started_;
    bool obstacles_changed_;
    bool emergency_stop_;
    bool new_goal_received_;
    bool hover_position_set_;
    bool is_replanning_;
    bool pre_offset_saved_;
    bool trajectory_planned_;
    bool initialization_complete_;
    bool is_emergency_recovery_;  // 标记是否是从emergency恢复
    bool formation_center_received_;
    bool recovery_position_calculated_;
    bool formation_transition_delay_started_;
    bool global_time_synced_ = false;   // 全局时间是否已同步
    bool has_global_sampling_time_ = false;
    bool judege_height = false;
    bool landing_judge = false;
    bool landing_position_recorded_ = false;

    ros::Time formation_transition_delay_start_;

    // 邻居相关 - 新增部分
    std::map<std::string, std::string> neighbor_states_;
    std::map<std::string, ros::Subscriber> neighbor_state_subs_;
    ros::Publisher neighbor_states_report_pub_;
    ros::Timer neighbor_report_timer_;
    std::shared_ptr<TopologyManager> topology_manager_;
    bool updateDynamicFormationOffsets(const std::string& formation_type);
    void formationCenterCallback(const geometry_msgs::Point::ConstPtr& msg, const std::string& neighbor_name);
    void publishFormationCenter();
    Eigen::Vector3d calculateAverageFormationCenter();
    Eigen::Vector3d calculateEmergencyRecoveryPosition();
    ros::Time global_traj_start_time_;  // 全局轨迹启动时间（所有无人机共享）
    ros::Subscriber global_traj_start_sub_;  // 订阅全局轨迹启动时间
    ros::Publisher  global_time_pub;
    void globalTrajStartTimeCallback(const std_msgs::Float64::ConstPtr& msg);
    double global_sampling_time_ = 0.0; 
};
#endif // UAV_STATE_MACHINE_H
