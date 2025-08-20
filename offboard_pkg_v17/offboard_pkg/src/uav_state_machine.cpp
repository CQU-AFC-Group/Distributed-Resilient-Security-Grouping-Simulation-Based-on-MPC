#include "offboard_pkg/uav_state_machine.h"
#include <std_msgs/String.h>
#define CONTROL_MODE 1 

UAVStateMachine::UAVStateMachine(ros::NodeHandle& nh, const std::string& uav_name)
    : nh_(nh), uav_name_(uav_name), current_state_(UAVState::IDLE),
      previous_state_(UAVState::IDLE), received_leader_path_(false),
      mission_started_(false), obstacles_changed_(false), distributed_started_(false),
      emergency_stop_(false), new_goal_received_(false) ,is_replanning_(false), hover_position_set_(false), in_formation_transition_(false),
      formation_change_requested_(false), pre_offset_saved_(false),trajectory_planned_(false), initialization_complete_(true),
      topology_manager_(nullptr), last_known_online_count_(0), current_online_count(0),target_formation_type_("circle"),
      is_emergency_recovery_(false), formation_center_received_(false){
    
    // 获取参数
    ros::NodeHandle pnh("~");
    formation_offsets = FormationCalculator::calculateFormationFromParams(nh, "formation");
    // 检查当前无人机是否在编队配置中
    if (formation_offsets.find(uav_name) != formation_offsets.end()) {
        Eigen::Vector3d offset = formation_offsets[uav_name];
        offset_x_ = offset.x();
        offset_y_ = offset.y();
        ROS_INFO("[%s] Using calculated formation offset: (%.3f, %.3f)", 
                 uav_name.c_str(), offset_x_, offset_y_);
    }
    local_offset_x = offset_x_;
    local_offset_y = offset_y_;
    pnh.getParam("neighbors", neighbors_);
    pnh.param("goal_tolerance", goal_tolerance_, 0.3);
    nh_.getParam("takeoff_height", takeoff_height_);
        
    // 创建定时器，定期上报邻居状态
    neighbor_report_timer_ = nh_.createTimer(ros::Duration(1.0), 
                                           &UAVStateMachine::neighborReportTimerCallback, this);

    // 初始化UAV Agent
    agent_ = std::make_unique<UAVAgent>(nh_, uav_name_, offset_x_, offset_y_, offset_x_, offset_y_);
    // 注册邻居
    for (const auto& neighbor : neighbors_) {
        agent_->registerNeighbor(neighbor);
    }
    // 订阅所有邻居的状态信息
    for (const auto& neighbor : neighbors_) {
        std::string topic_name = "/" + neighbor + "/state";
        ros::Subscriber sub = nh_.subscribe<std_msgs::String>(topic_name, 1, boost::bind(&UAVStateMachine::neighborStateCallback, this, _1, neighbor));
        neighbor_state_subs_[neighbor] = sub;
        neighbor_states_[neighbor] = "UNKNOWN";
        ROS_INFO("[%s] Subscribed to neighbor %s state topic: %s", 
                 uav_name_.c_str(), neighbor.c_str(), topic_name.c_str());
    }
     // 订阅所有邻居的编队中心
    for (const auto& neighbor : neighbors_) {
        std::string topic_name = "/" + neighbor + "/formation_center";
        ros::Subscriber sub = nh_.subscribe<geometry_msgs::Point>(topic_name, 1, 
            boost::bind(&UAVStateMachine::formationCenterCallback, this, _1, neighbor));
        formation_center_subs_[neighbor] = sub;
        ROS_INFO("[%s] Subscribed to neighbor %s formation center: %s", 
                 uav_name_.c_str(), neighbor.c_str(), topic_name.c_str());
    }
    
    // // 设置初始目标
    // 加载起点，添加默认值
    std::vector<double> start_point;
    if (pnh.getParam("/start_point", start_point) && start_point.size() == 3) {
        start_position_ = Eigen::Vector3d(start_point[0] + offset_x_ - agent_->offset_x0,
                                         start_point[1] + offset_y_ - agent_->offset_y0,
                                         start_point[2]);
        ROS_WARN("[%s] Start point: %.2f,%.2f,%.2f", uav_name_.c_str(), start_point[0], start_point[1], start_point[2]);
    } else {
        ROS_WARN("[%s] Start point not found in config, using default [0,0,1.5]", uav_name_.c_str());
    }
    
    // 加载终点，添加默认值
    std::vector<double> goal_point;
    if (pnh.getParam("/goal_point", goal_point) && goal_point.size() == 3) {
        current_goal_ = Eigen::Vector3d(goal_point[0] + offset_x_ - agent_->offset_x0,
                                       goal_point[1] + offset_y_ - agent_->offset_y0,
                                       goal_point[2]);
        temp_current_goal_ = Eigen::Vector3d(goal_point[0], goal_point[1], goal_point[2]);
    } else {
        ROS_WARN("[%s] Goal point not found in config, using default [5,5,1.5]", uav_name_.c_str());
    }

    // 加载到达时间
    pnh.param("/set_time", t_reach, 100.0); 
    
    // 初始化ROS发布者和订阅者
    state_pub_ = nh_.advertise<std_msgs::String>("/" + uav_name_ + "/state", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/" + uav_name_ + "/status", 1);

    // 全局控制
    terminal_takeoff_sub_ = nh_.subscribe<std_msgs::Bool>("/terminal_takeoff", 1,
        &UAVStateMachine::terminalTakeoffCallback, this);
    distuibuted_control_sub_ = nh_.subscribe<std_msgs::Bool>("/start_distributed_control", 1,
        &UAVStateMachine::startControlCallback, this);
    new_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("/new_goal", 1,
        &UAVStateMachine::newGoalCallback, this);
    emergency_sub_ = nh_.subscribe<std_msgs::Bool>("/emergency", 1,
        &UAVStateMachine::emergencyCallback, this);
    reset_sub_ = nh_.subscribe<std_msgs::Bool>("/reset", 1,
        &UAVStateMachine::resetCallback, this);
    mission_start_sub_ = nh_.subscribe<std_msgs::Bool>("/mission_start", 1,
            &UAVStateMachine::missionCallback, this);
    formation_change_sub_ = nh_.subscribe<std_msgs::String>("/formation_change", 1,
            &UAVStateMachine::formationChangeCallback, this);
    global_traj_start_sub_ = nh.subscribe("/uav0/global_traj_start_time", 10,
        &UAVStateMachine::globalTrajStartTimeCallback, this);
    
    // 初始化邻居状态订阅和发布
    neighbor_states_report_pub_ = nh_.advertise<std_msgs::String>("/" + uav_name_ + "/neighbor_states_report", 1);
    // 单个无人机状态控制
    individual_mission_start_sub_ = nh_.subscribe<std_msgs::Bool>("/" + uav_name_ + "/mission_start", 1, &UAVStateMachine::missionCallback, this);
    individual_terminal_takeoff_sub_ = nh_.subscribe<std_msgs::Bool>("/" + uav_name_ + "/terminal_takeoff", 1, &UAVStateMachine::terminalTakeoffCallback, this);
    individual_emergency_sub_ = nh_.subscribe<std_msgs::Bool>("/" + uav_name_ + "/emergency", 1, &UAVStateMachine::emergencyCallback, this);
    formation_center_pub_ = nh_.advertise<geometry_msgs::Point>("/" + uav_name_ + "/formation_center", 1);
    global_time_pub = nh_.advertise<std_msgs::Float64>("/" + uav_name_ + "/global_traj_start_time", 10);

    state_enter_time_ = ros::Time::now();
    last_obstacle_check_ = ros::Time::now();
    ROS_INFO("[%s] State machine initialized", uav_name_.c_str());
    initialization_complete_ = true;
}

UAVStateMachine::~UAVStateMachine() {}

void UAVStateMachine::update(double dt) {
    checkAndHandleFormationChange(); // 实时检测队形变换与拓扑信息
    // 根据当前状态执行相应逻辑
    switch (current_state_) {
        case UAVState::IDLE:
            handleIdleState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: IDLE", uav_name_.c_str());
            break;
        case UAVState::INITIALIZING:
            handleInitializingState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: INITIALIZING", uav_name_.c_str());
            break;
        case UAVState::PRE_PLANNING:
            handlePrePlanningState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: PRE_PLANNING", uav_name_.c_str());
            break;
        case UAVState::TAKING_OFF:
            handleTakingOffState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: TAKING_OFF",uav_name_.c_str());
            break;
        case UAVState::HOVERING:
            handleHoveringState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: HOVERING",uav_name_.c_str());
            break;
        case UAVState::EXECUTING:
            handleExecutingState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: EXECUTING",uav_name_.c_str());
            break;
        case UAVState::EMERGENCY:
            handleEmergencyState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: EMERGENCY",uav_name_.c_str());
            break;
        case UAVState::LANDING:
            handleLandingState(dt);
            ROS_WARN_THROTTLE(2.0, "[%s] UAV State: LANDING",uav_name_.c_str());
            break;
    }
    // 发布状态信息
    publishStateInfo();
    // 检查紧急停止
    if (emergency_stop_ && current_state_ != UAVState::EMERGENCY) {
        handleEvent(UAVEvent::EMERGENCY_STOP);
    }
}

void UAVStateMachine::handleEvent(UAVEvent event) {
    switch (current_state_) {
        case UAVState::IDLE:
            if (event == UAVEvent::START_MISSION) {
                transitionTo(UAVState::INITIALIZING);
                mission_started_ = false;
                judege_height = false;
            }
            break;
            
        case UAVState::INITIALIZING:
            if (event == UAVEvent::OBSTACLES_LOADED) {
                transitionTo(UAVState::PRE_PLANNING);
            }
            break;
        case UAVState::PRE_PLANNING:
            // 等待终端起飞命令
            if (event == UAVEvent::PRE_PLANNING_COMPLETE) {
                     // 检查是否是悬停状态下的队形变换
                if (previous_state_ == UAVState::HOVERING) 
                {
                    if (in_formation_transition_) {
                        // 悬停状态下的队形变换完成，重置所有标志位
                        in_formation_transition_ = false;
                        pre_offset_saved_ = false;
                        formation_change_requested_ = false;
                        ROS_INFO("[%s] Formation transition completed in hovering state", uav_name_.c_str());
                    }
                    // 如果是重新规划，完成后回到悬停状态
                    transitionTo(UAVState::HOVERING);
                    ROS_INFO("[%s] Replanning completed, returning to hovering", uav_name_.c_str());
                } 
                else if(in_formation_transition_){
                    // 队形变换的重规划完成后，自动开始执行
                    ROS_INFO("[%s] Formation change replanning completed, returning to hovering", 
                        uav_name_.c_str());
                    in_formation_transition_ = false;
                    pre_offset_saved_ = false;
                    formation_change_requested_ = false;
                    transitionTo(UAVState::HOVERING);  // 改为悬停状态
                }
            else {
                ROS_INFO("[%s] Initial planning completed, waiting for takeoff command", uav_name_.c_str());
            }
            }
            break;
        case UAVState::TAKING_OFF:
            if (event == UAVEvent::TAKEOFF_COMPLETE) {
                transitionTo(UAVState::HOVERING);
            }
            break;
            
        case UAVState::HOVERING:
            if (event == UAVEvent::START_EXECUTION) {
                transitionTo(UAVState::EXECUTING);
            }
            else if (event == UAVEvent::NEW_GOAL_RECEIVED) {
                transitionTo(UAVState::PRE_PLANNING);  // 接收到新目标后转到规划状态
            }
            break;
            
        case UAVState::EXECUTING:
            if (event == UAVEvent::GOAL_REACHED) {
                // 检查是否是队形变换完成
                if (in_formation_transition_) {
                    // 队形变换完成，标记结束，重置标志位
                    in_formation_transition_ = false;
                    pre_offset_saved_ = false;
                    formation_change_requested_ = false;
                    ROS_INFO("[%s] Formation transition execution completed", uav_name_.c_str());
                }
                transitionTo(UAVState::HOVERING);// 等待其他命令接收
            } 
            break;
        default:
            break;
    }
    // 全局事件处理
    if (event == UAVEvent::EMERGENCY_STOP) {
        transitionTo(UAVState::LANDING);
    }
}

void UAVStateMachine::handleIdleState(double dt) {
    ROS_WARN_THROTTLE(2.0, "[%s] WAITING FOR MISSION", uav_name_.c_str());
    if(mission_started_)
    {
        handleEvent(UAVEvent::START_MISSION);
    }
    else{
        // 创建上锁服务请求
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;  // false表示上锁，true表示解锁
        // 检查无人机是否处于解锁状态，若已解锁则发送上锁指令
        if (agent_->current_state_.armed) {  // 直接使用mavros/state中的armed状态
            // 构造上锁请求（value=false表示上锁）
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;
            // 调用已初始化的arming_client_服务
            if (agent_->arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("[%s] successfully armed", uav_name_.c_str());
            } else {
                ROS_WARN_THROTTLE(1.0, "[%s] failed to arm", uav_name_.c_str());
            }
        }
    }
}

void UAVStateMachine::handleInitializingState(double dt) {
    // 执行初始化逻辑
    if (loadObstaclesAndWaypoints()) {
        handleEvent(UAVEvent::OBSTACLES_LOADED);
    }
}

void UAVStateMachine::handleTakingOffState(double dt) {
    static bool preheated = false;
    static bool armed = false;
    static int preheat_count = 0;
    static ros::Time last_time;
    
    // 步骤1: 预热setpoint
    if (!preheated) {
        if (preheat_count == 0) {
            ROS_INFO("[%s] Preheating setpoints...", uav_name_.c_str());
            last_time = ros::Time::now();
        }
        
        if (preheat_count < 100) {
            geometry_msgs::PoseStamped warmup_pose;
            warmup_pose.header.stamp = ros::Time::now();
            warmup_pose.header.frame_id = "map";
            
            warmup_pose.pose.position.x = agent_->pose_.pose.position.x;
            warmup_pose.pose.position.y = agent_->pose_.pose.position.y;
            warmup_pose.pose.position.z = agent_->pose_.pose.position.z;
            warmup_pose.pose.orientation = agent_->pose_.pose.orientation;
            
            agent_->local_pos_pub_.publish(warmup_pose);
            preheat_count++;
            return;
        } else {
            preheated = true;
            ROS_INFO("[%s] Setpoint preheating complete", uav_name_.c_str());
        }
    }
    
    // 步骤2: 解锁并切换到OFFBOARD模式
    if (!armed) {
        #if CONTROL_MODE == 1
        if (!agent_->armAndSetMode()) {
            ROS_ERROR("[%s] Failed to arm and set mode", uav_name_.c_str());
            return;
        }
        #endif
        armed = true;
        ROS_INFO("[%s] Armed and switched to OFFBOARD mode", uav_name_.c_str());
    }

    // 步骤3: 发送目标高度的位置指令（区分重启和正常起飞）
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "map";
    target_pose.pose.orientation = agent_->pose_.pose.orientation;
    if (is_emergency_recovery_) {
        // 紧急恢复：起飞到编队中心位置
        Eigen::Vector3d recovery_position = calculateEmergencyRecoveryPosition();
        target_pose.pose.position.x = recovery_position.x() - local_offset_x;
        target_pose.pose.position.y = recovery_position.y() - local_offset_y;
        target_pose.pose.position.z = recovery_position.z();
        ROS_INFO("[%s] Emergency recovery takeoff to: (%.3f, %.3f, %.3f)", 
                 uav_name_.c_str(), recovery_position.x(), recovery_position.y(), recovery_position.z());
    } else {
        // 正常起飞：起飞到原点
        target_pose.pose.position.x = 0;
        target_pose.pose.position.y = 0;
        target_pose.pose.position.z = takeoff_height_;
        ROS_INFO("[%s] Normal takeoff to: (0, 0, %.3f)", uav_name_.c_str(), takeoff_height_);
    }
    agent_->local_pos_pub_.publish(target_pose);
    
    // 检查是否达到目标高度
    double current_z = agent_->pose_.pose.position.z;
    if (std::abs(current_z - takeoff_height_) < 0.1) {
        ROS_INFO("[%s] Reached target height: %.2f meters", uav_name_.c_str(), current_z);
        // 重置状态变量，以便下次起飞
        preheated = false;
        armed = false;
        preheat_count = 0;
        // 触发状态转换
        handleEvent(UAVEvent::TAKEOFF_COMPLETE);
    }
}

void UAVStateMachine::handleHoveringState(double dt) {
    geometry_msgs::PoseStamped hover_pose;
    hover_pose.header.frame_id = "map";
    hover_pose.header.stamp = ros::Time::now();
    // 根据前一个状态决定悬停位置
    if (previous_state_ == UAVState::EXECUTING) {
        // 如果是从执行状态转换过来的，说明刚到达目标，在目标位置悬停
        hover_pose.pose.position.x = current_goal_.x() - local_offset_x;
        hover_pose.pose.position.y = current_goal_.y() - local_offset_y;
        hover_pose.pose.position.z = current_goal_.z();
    } 
    else if (previous_state_ == UAVState::TAKING_OFF || previous_state_ == UAVState::PRE_PLANNING) {
        // 如果是刚起飞，在起飞位置悬停
        hover_pose.pose.position.x = start_position_.x() - local_offset_x;
        hover_pose.pose.position.y = start_position_.y() - local_offset_y;
        hover_pose.pose.position.z = takeoff_height_;
    } 
    // 重新启动加入时，直接以编队中心为基准计算悬停位置
    if (is_emergency_recovery_ && previous_state_ == UAVState::TAKING_OFF) {
        // 紧急恢复状态检查 - 在规划前更新起始位置
        ROS_INFO("[%s] Emergency recovery: updating start position before planning", uav_name_.c_str());
        is_replanning_ = true;
        transitionTo(UAVState::PRE_PLANNING);  // 先规划再执行
        // 从邻居获取的平均编队中心（期望的队形中心）
        Eigen::Vector3d target_center = calculateAverageFormationCenter();
        hover_pose.pose.position.x = target_center.x() + offset_x_ - local_offset_x;  // 中心+自身偏移
        hover_pose.pose.position.y = target_center.y() + offset_y_ - local_offset_y;
        hover_pose.pose.position.z = target_center.z();
        is_emergency_recovery_ = false;  // 重置标志，避免重复触发
    } 
    hover_pose.pose.orientation = agent_->pose_.pose.orientation;
    agent_->local_pos_pub_.publish(hover_pose);

    // 检查是否有新目标接收
    if (new_goal_received_) {
        new_goal_received_ = false;
        is_replanning_ = true;
        handleEvent(UAVEvent::NEW_GOAL_RECEIVED);  //处理新目标事件
        return;
    }
    
    if(distributed_started_)
    {
        handleEvent(UAVEvent::START_EXECUTION);
        distributed_started_ = false;
    }
    // 发布编队中心供其他无人机参考
    publishFormationCenter();
}

void UAVStateMachine::handleExecutingState(double dt) {
    // 重置标志位
    agent_->loadObstaclesFromParamServer(); // 实时加载更新障碍物
    agent_->updateTrajectory(dt);   // 轨迹更新+MPC控制
    agent_->publishAgentState();    // 路径rviz可视化
    agent_->publishObstacles();     // 障碍物rviz实时可视化
    // 检查是否到达目标
    if (isGoalReached()) {
        handleEvent(UAVEvent::GOAL_REACHED);
    }
    // 发布编队中心供其他无人机参考
    publishFormationCenter();
}

void UAVStateMachine::handleEmergencyState(double dt) {
    // 紧急状态处理
    ROS_WARN_THROTTLE(1.0, "[%s] Emergency state active", uav_name_.c_str());
}

void UAVStateMachine::handleLandingState(double dt) {
    // 降落逻辑
    ROS_INFO_THROTTLE(2.0, "[%s] Landing...", uav_name_.c_str());
    Eigen::Vector3d landing_initial_position_;
    geometry_msgs::PoseStamped target_pose;
    if (!landing_position_recorded_) {
        // 记录触发降落时的位置（减去偏移量，确保是机体实际位置）
        landing_initial_position_.x() = agent_->pose_.pose.position.x - local_offset_x;
        landing_initial_position_.y() = agent_->pose_.pose.position.y - local_offset_y;
        landing_initial_position_.z() = agent_->pose_.pose.position.z; // 记录当前高度，用于平滑降落
        landing_position_recorded_ = true; // 标记已记录，避免重复更新
        ROS_INFO("[%s] Landing target position locked: (%.2f, %.2f, 0)", 
                 uav_name_.c_str(), landing_initial_position_.x(), landing_initial_position_.y());
    }

    // 始终使用记录的初始位置作为降落目标（z轴逐渐降为0）
    target_pose.pose.position.x = landing_initial_position_.x();
    target_pose.pose.position.y = landing_initial_position_.y();
    
    // 添加z轴平滑下降逻辑（避免直接跳变到0）
    double current_z = agent_->pose_.pose.position.z;
    double target_z = std::max(0.0, current_z - 0.05); // 每次循环降低0.05m，最低到0
    target_pose.pose.position.z = target_z;
    if (target_z == 0 && !judege_height) {
        emergency_stop_ = false;
        mission_started_ = false;
        landing_judge = false;
        judege_height = true;
        transitionTo(UAVState::IDLE);
    }
}

void UAVStateMachine::transitionTo(UAVState new_state) {
    if (new_state != current_state_) {
        onExitState(current_state_);
        previous_state_ = current_state_;
        current_state_ = new_state;
        state_enter_time_ = ros::Time::now();
        onEnterState(new_state);
    }
}

void UAVStateMachine::onEnterState(UAVState state) {
    switch (state) {
        case UAVState::EXECUTING:
            resetTrajectory();
        case UAVState::PRE_PLANNING:
            ROS_INFO("[%s] Starting trajectory planning", uav_name_.c_str());
            break;
        default:
            break;
    }
}

void UAVStateMachine::onExitState(UAVState state) {
    // 状态退出时的清理工作
}

bool UAVStateMachine::loadObstaclesAndWaypoints() {
    // 重新加载障碍物信息
    agent_->loadObstaclesFromParamServer();
    return true;
}


bool UAVStateMachine::planTrajectory() {
    std::vector<Obstacle> obstacles = agent_->getObstacles();
    
    // 所有无人机使用统一的规划逻辑
    planner_ = std::make_unique<CustomTrajectoryPlanner>();
    planner_->setObstacles(obstacles);

    // 统一使用 generateTrajectory
    bool success = planner_->generateTrajectory(start_position_, current_goal_, t_reach);
    
    if (success) {
        planner_->startPublishingPathPoints(nh_, uav_name_);
        agent_->setTrajectoryPlanner(planner_.get());
        return true;
    }
    
    ROS_ERROR("[%s] Failed to generate trajectory", uav_name_.c_str());
    return false;
}

bool UAVStateMachine::isGoalReached() {
    Eigen::Vector3d current_pos(agent_->pose_.pose.position.x,
                               agent_->pose_.pose.position.y,
                               agent_->pose_.pose.position.z);
    
    double distance = (current_pos - current_goal_).norm();
    return distance < goal_tolerance_;
}

void UAVStateMachine::publishStateInfo() {
    std_msgs::String state_msg;
    state_msg.data = std::to_string(static_cast<int>(current_state_));
    state_pub_.publish(state_msg);
}

void UAVStateMachine::resetTrajectory() {
    agent_->resetTrajectoryTime();
}

// ROS回调函数实现
void UAVStateMachine::missionCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && is_emergency_recovery_) {
         // 重置distributed_started_为false，确保重新加入后等待指令
         distributed_started_ = false;
    }
    mission_started_ = msg->data;
}

void UAVStateMachine::formationChangeCallback(const std_msgs::String::ConstPtr& msg) {
    target_formation_type_ = msg->data;
    formation_change_requested_ = true;
    ROS_INFO("[%s] Formation change requested: %s", 
             uav_name_.c_str(), target_formation_type_.c_str());
}

void UAVStateMachine::startControlCallback(const std_msgs::Bool::ConstPtr& msg) {
    distributed_started_ = msg->data;
}

void UAVStateMachine::terminalTakeoffCallback(const std_msgs::Bool::ConstPtr& msg) {
    ROS_INFO("[%s] Waiting for terminal takeoff command", uav_name_.c_str());
    if (msg->data && current_state_ == UAVState::PRE_PLANNING) {
        ROS_INFO("[%s] Terminal takeoff command received", uav_name_.c_str());
        transitionTo(UAVState::TAKING_OFF);
    }
}

void UAVStateMachine::newGoalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    temp_current_goal_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
    current_goal_ = Eigen::Vector3d(temp_current_goal_[0] + offset_x_ - agent_->offset_x0,
        temp_current_goal_[1] + offset_y_ - agent_->offset_y0,
        temp_current_goal_[2]);
    new_goal_received_ = true;
    ROS_INFO("[%s] New goal received: (%.2f, %.2f, %.2f)", 
             uav_name_.c_str(), msg->x, msg->y, msg->z);
}

void UAVStateMachine::emergencyCallback(const std_msgs::Bool::ConstPtr& msg) {
    emergency_stop_ = msg->data;
    is_emergency_recovery_ = true;
    distributed_started_ = false;
}

void UAVStateMachine::globalTrajStartTimeCallback(const std_msgs::Float64::ConstPtr& msg) {
    global_sampling_time_ = msg->data;
    has_global_sampling_time_ = true;
    ROS_INFO("[%s] Synced global trajectory start time: %.3f",
             uav_name_.c_str(), global_traj_start_time_.toSec());
}

void UAVStateMachine::resetCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        handleEvent(UAVEvent::RESET_REQUEST);
    }
}

// 邻居状态相关方法实现
void UAVStateMachine::neighborStateCallback(const std_msgs::String::ConstPtr& msg, const std::string& neighbor_name) {
    int state_int = std::stoi(msg->data);
    UAVState neighbor_state = static_cast<UAVState>(state_int);
    std::string state_str = stateToString(neighbor_state);
    
    // 更新邻居状态
    neighbor_states_[neighbor_name] = state_str;
    
    ROS_DEBUG("[%s] Updated neighbor %s state to: %s", 
              uav_name_.c_str(), neighbor_name.c_str(), state_str.c_str());
}

void UAVStateMachine::reportNeighborStates() {
    if (neighbor_states_.empty()) {
        return;
    }
    // 构造上报消息格式: "uav0:uav1=HOVERING,uav9=EXECUTING"
    std::stringstream ss;
    ss << uav_name_ << ":";
    bool first = true;
    for (const auto& state_pair : neighbor_states_) {
        if (!first) {
            ss << ",";
        }
        ss << state_pair.first << "=" << state_pair.second;
        first = false;
    }
    std_msgs::String report_msg;
    report_msg.data = ss.str();
    neighbor_states_report_pub_.publish(report_msg);
}

void UAVStateMachine::neighborReportTimerCallback(const ros::TimerEvent& event) {
    reportNeighborStates();
}

std::string UAVStateMachine::stateToString(UAVState state) {
    switch (state) {
        case UAVState::IDLE: return "IDLE";
        case UAVState::INITIALIZING: return "INITIALIZING";
        case UAVState::PRE_PLANNING: return "PRE_PLANNING";
        case UAVState::TAKING_OFF: return "TAKING_OFF";
        case UAVState::HOVERING: return "HOVERING";
        case UAVState::EXECUTING: return "EXECUTING";
        case UAVState::EMERGENCY: return "EMERGENCY";
        case UAVState::LANDING: return "LANDING";
        default: return "UNKNOWN";
    }
}

void UAVStateMachine::handlePrePlanningState(double dt) {
    geometry_msgs::PoseStamped hover_pose;
    hover_pose.header.frame_id = "map";
    hover_pose.header.stamp = ros::Time::now();
    // 如果是从悬停状态进入的重新规划，更新起始位置为当前位置
    if (is_replanning_) {
        // 重置轨迹规划标志
        trajectory_planned_ = false;
        if (in_formation_transition_) {
            // 队形变换的重规划 - 使用预设的起点和目标点
            ROS_INFO("[%s] Formation transition replanning", uav_name_.c_str());
            hover_pose.pose.position.x = agent_->pose_.pose.position.x - pre_offset_x;
            hover_pose.pose.position.y = agent_->pose_.pose.position.y - pre_offset_y;
            hover_pose.pose.position.z = agent_->pose_.pose.position.z;
            
        } 
        else {
            // 普通重规划 - 使用当前位置作为起点
            ROS_INFO("[%s] Normal replanning", uav_name_.c_str());
            start_position_ = Eigen::Vector3d(
                agent_->pose_.pose.position.x,
                agent_->pose_.pose.position.y,
                agent_->pose_.pose.position.z);
            hover_pose.pose.position.x = agent_->pose_.pose.position.x - offset_x_;
            hover_pose.pose.position.y = agent_->pose_.pose.position.y - offset_y_;
            hover_pose.pose.position.z = agent_->pose_.pose.position.z;
        }
        hover_pose.pose.orientation = agent_->pose_.pose.orientation;
        is_replanning_ = false;
    }

    // **持续发送悬停指令，保持位置控制**
    if (previous_state_ == UAVState::HOVERING || previous_state_ == UAVState::EXECUTING){
        agent_->local_pos_pub_.publish(hover_pose);
    }
    // **只在未规划时进行轨迹规划**
    if (!trajectory_planned_) {
        if (planTrajectory()) {
            trajectory_planned_ = true;  // 标记已规划
            handleEvent(UAVEvent::PRE_PLANNING_COMPLETE);
            ROS_INFO("[%s] Trajectory planning completed.", uav_name_.c_str());
        } else {
            // 规划失败，终端提示
            if ((ros::Time::now() - state_enter_time_).toSec() > 5.0) {
                ROS_INFO_THROTTLE(5.0, "[%s] Trajectory planning failed", uav_name_.c_str());
            }
        }
    } 
    else {
        // 已经规划完成，等待状态转换
        ROS_INFO_THROTTLE(2.0, "[%s] Trajectory ready, waiting for next command", uav_name_.c_str());
    }
}

void UAVStateMachine::checkAndHandleFormationChange() {
    detectTopologyChange();
    if (formation_change_requested_) {
        distributed_started_ = false;
        formation_change_requested_ = false;
        in_formation_transition_ = true;
        // 保存旧偏移量
        if (!pre_offset_saved_) {
            pre_offset_x = offset_x_;
            pre_offset_y = offset_y_;
            pre_offset_saved_ = true;
            ROS_INFO("[%s] Saved old formation offset: (%.3f, %.3f)", 
                    uav_name_.c_str(), pre_offset_x, pre_offset_y);
        }
        
        // 更新编队偏移量
        if (!updateDynamicFormationOffsets(target_formation_type_)) {
            ROS_ERROR("[%s] Failed to update to dynamic %s formation", 
                    uav_name_.c_str(), target_formation_type_.c_str());
            offset_x_ = pre_offset_x;
            offset_y_ = pre_offset_y;
            return;
        }

        ROS_INFO("[%s] Updated to formation offset: (%.3f, %.3f)", 
                uav_name_.c_str(), offset_x_, offset_y_);

        // 根据当前状态选择不同的队形变换策略
        if (current_state_ == UAVState::HOVERING) {
            setupHoveringFormationTransition();
            // 触发重新规划
            is_replanning_ = true;
            transitionTo(UAVState::PRE_PLANNING);
        } 
        else if (current_state_ == UAVState::EXECUTING) {
            double current_time = 0;
            // 确定当前使用的采样时间
            if (uav_name_ == "uav0") {
                // uav0：使用自身计算的时间，并发布给其他无人机
                ros::Time global_start = ros::Time(agent_->traj_start_time_);
                current_time = (ros::Time::now() - global_start).toSec();
                // 发布当前采样时间
                std_msgs::Float64 msg;
                msg.data = current_time;
                global_time_pub.publish(msg);
                setupExecutingFormationTransition(current_time);
                // 触发重新规划
                is_replanning_ = true;
                transitionTo(UAVState::PRE_PLANNING);
            } // 其他无人机：优先用uav0的时间
            else{
                if (has_global_sampling_time_) {
                    current_time = global_sampling_time_; // 正常使用uav0时间
                    setupExecutingFormationTransition(current_time);
                    // 触发重新规划
                    is_replanning_ = true;
                    transitionTo(UAVState::PRE_PLANNING);
                } 
                else{
                    formation_change_requested_ = true;
                }
            }   
        }
    }
}

void UAVStateMachine::setupHoveringFormationTransition() {
    ROS_INFO("[%s] Setting up formation transition in HOVERING state", uav_name_.c_str());
    
    // 悬停状态：使用当前位置推算编队中心的经典方法
    Eigen::Vector3d current_pos(
        agent_->pose_.pose.position.x,
        agent_->pose_.pose.position.y, 
        agent_->pose_.pose.position.z
    );
    
    // 推算当前编队中心位置（当前位置减去旧偏移量）
    Eigen::Vector3d estimated_center = Eigen::Vector3d(
        current_pos.x() - pre_offset_x,
        current_pos.y() - pre_offset_y,
        current_pos.z()
    );
    
    // 新编队中该无人机的目标位置（编队中心 + 新偏移量）
    Eigen::Vector3d new_formation_position = Eigen::Vector3d(
        estimated_center.x() + offset_x_,
        estimated_center.y() + offset_y_,
        estimated_center.z()
    );
    
    // 最终目标位置（根据编队类型调整）
    Eigen::Vector3d final_target_position;
    if (target_formation_type_ == "line") {
        final_target_position = Eigen::Vector3d(
            temp_current_goal_[0],
            temp_current_goal_[1] + offset_y_,
            temp_current_goal_[2]
        );
    } else if (target_formation_type_ == "circle") {
        final_target_position = Eigen::Vector3d(
            temp_current_goal_[0] + offset_x_,
            temp_current_goal_[1] + offset_y_,
            temp_current_goal_[2]
        );
    }
    
    // 设置轨迹规划的起点和终点
    start_position_ = new_formation_position;  // 新编队中的当前位置
    current_goal_ = final_target_position;     // 新编队中的目标位置
    
    ROS_INFO("[%s] HOVERING formation transition:", uav_name_.c_str());
    ROS_INFO("  Current position:     (%.3f, %.3f, %.3f)", 
             current_pos.x(), current_pos.y(), current_pos.z());
    ROS_INFO("  Estimated center:     (%.3f, %.3f, %.3f)", 
             estimated_center.x(), estimated_center.y(), estimated_center.z());
    ROS_INFO("  New formation pos:    (%.3f, %.3f, %.3f)", 
             new_formation_position.x(), new_formation_position.y(), new_formation_position.z());
    ROS_INFO("  Final target:         (%.3f, %.3f, %.3f)", 
             final_target_position.x(), final_target_position.y(), final_target_position.z());
}

void UAVStateMachine::setupExecutingFormationTransition(double current_time) {
    ROS_INFO("[%s] Setting up formation transition in EXECUTING state", uav_name_.c_str());
    
    // 执行状态：使用参考轨迹位置作为统一编队中心
    Eigen::Vector3d reference_center = calculateReferenceFormationCenter(current_time);
    
    // 计算该无人机在新编队中的目标位置
    Eigen::Vector3d new_formation_position;
    if (target_formation_type_ == "line") {
        new_formation_position = Eigen::Vector3d(
            reference_center.x(),                    // 直线编队：X保持中心
            reference_center.y() + offset_y_,        // Y方向排列
            reference_center.z()
        );
    } else if (target_formation_type_ == "circle") {
        new_formation_position = Eigen::Vector3d(
            reference_center.x() + offset_x_,        // 圆形编队：XY都有偏移
            reference_center.y() + offset_y_,
            reference_center.z()
        );
    } 
    start_position_ = new_formation_position;
    ROS_INFO("[%s] EXECUTING formation transition:", uav_name_.c_str());
    ROS_INFO("  Reference center:     (%.3f, %.3f, %.3f)", 
             reference_center.x(), reference_center.y(), reference_center.z());
    ROS_INFO("  start_position_:     (%.3f, %.3f, %.3f)", 
             start_position_.x(), start_position_.y(), start_position_.z());
}

Eigen::Vector3d UAVStateMachine::calculateReferenceFormationCenter(double current_time) {
    // 仅在EXECUTING状态下使用参考轨迹
    Eigen::Vector3d formation_center = Eigen::Vector3d::Zero();
   
    static ros::Time wait_start = ros::Time::now();
    if (current_state_ == UAVState::EXECUTING && agent_->planner_ && agent_->planner_->isTrajectoryGenerated()) {
        Eigen::Vector3d ref_pos, ref_vel;
        if (agent_->planner_->sampleTrajectory(current_time, ref_pos, ref_vel)) {
            // 从参考轨迹位置推算编队中心
            formation_center = Eigen::Vector3d(
                ref_pos.x() - pre_offset_x,
                ref_pos.y() - pre_offset_y,
                ref_pos.z()
            );
            ROS_INFO("[%s] Reference formation center from trajectory: (%.3f, %.3f, %.3f)", 
                     uav_name_.c_str(), formation_center.x(), formation_center.y(), formation_center.z());   
            ROS_WARN("[%s] Trajectory sampling time: %.3f seconds (since trajectory start)",
            uav_name_.c_str(), current_time);
        }
    }
    return formation_center;
}


// 动态更新编队偏移量的实现
bool UAVStateMachine::updateDynamicFormationOffsets(const std::string& formation_type) {
    if (!topology_manager_) {
        ROS_ERROR("[%s] TopologyManager not set, cannot use dynamic formation", uav_name_.c_str());
        return false;
    }
    
    // 通过 FormationManager 计算动态编队，传入 TopologyManager 信息
    std::map<std::string, Eigen::Vector3d> dynamic_offsets = 
        FormationCalculator::calculateDynamicFormationFromTopology(
            topology_manager_.get(), formation_type, nh_, "formation");
    
    // 检查当前无人机是否在动态编队中
    if (dynamic_offsets.find(uav_name_) != dynamic_offsets.end()) {
        Eigen::Vector3d new_offset = dynamic_offsets[uav_name_];
        
        double old_offset_x = offset_x_;
        double old_offset_y = offset_y_;
        
        offset_x_ = new_offset.x();
        offset_y_ = new_offset.y();
        
        agent_->updateOffset(offset_x_, offset_y_);
        
        int online_count = topology_manager_->getOnlineCount();
        ROS_INFO("[%s] Dynamic %s formation (%d UAVs): offset (%.3f,%.3f) -> (%.3f,%.3f)", 
                 uav_name_.c_str(), formation_type.c_str(), online_count,
                 old_offset_x, old_offset_y, offset_x_, offset_y_);
        current_goal_ = Eigen::Vector3d(
        temp_current_goal_[0] + offset_x_ - agent_->offset_x0,
        temp_current_goal_[1] + offset_y_ - agent_->offset_y0,
        temp_current_goal_[2]);
        // temp_current_goal_ = current_goal_;
        return true;
    } 
    else {
        ROS_WARN("[%s] UAV not found in dynamic formation (offline?)", uav_name_.c_str());
        return false;
    }
}

bool UAVStateMachine::detectTopologyChange() {
    if (!topology_manager_ || !initialization_complete_) return false;
    // EXECUTING状态下不检测拓扑变化
    
    // 获取当前在线无人机数量
    int current_online_count = topology_manager_->getOnlineCount();

    // 添加状态检查，只在特定状态下才检测拓扑变化
    if (current_state_ == UAVState::INITIALIZING ||
        current_state_ == UAVState::PRE_PLANNING ||
        current_state_ == UAVState::TAKING_OFF) {
        return false;  // 初始化阶段不检测拓扑变化
    }
    
    // 检查是否有变化
    if (current_online_count != last_known_online_count_) {
        ROS_INFO("[%s] Topology change detected: %d -> %d online UAVs", 
                 uav_name_.c_str(), last_known_online_count_, current_online_count);
        // 在IDLE状态下直接更新偏移量，不触发队形变换流程
        if (current_state_ == UAVState::IDLE) {
            if (updateDynamicFormationOffsets(target_formation_type_)) {
                ROS_INFO("[%s] Updated formation offset in IDLE state: (%.3f, %.3f)", 
                         uav_name_.c_str(), offset_x_, offset_y_);
            }
        }
        // 如果是紧急恢复且检测到拓扑增加（自己重新上线）
        else if (is_emergency_recovery_ && current_online_count > last_known_online_count_) {
            // 使用最新拓扑信息计算恢复位置
            calculateEmergencyRecoveryPosition();
        }
        else{
            formation_change_requested_ = true;
            pre_offset_saved_ = false;
        }
        last_known_online_count_ = current_online_count;
        last_topology_change_time_ = ros::Time::now();
        return true;
    }
    return false;
}

void UAVStateMachine::publishFormationCenter() {
    // 计算当前编队中心位置（当前位置减去偏移量）
    Eigen::Vector3d current_formation_center(
        agent_->pose_.pose.position.x - local_offset_x,
        agent_->pose_.pose.position.y - local_offset_y,
        agent_->pose_.pose.position.z
    );
    geometry_msgs::Point center_msg;
    center_msg.x = current_formation_center.x();
    center_msg.y = current_formation_center.y();
    center_msg.z = current_formation_center.z();
    formation_center_pub_.publish(center_msg);
}

void UAVStateMachine::formationCenterCallback(const geometry_msgs::Point::ConstPtr& msg, const std::string& neighbor_name) {
    Eigen::Vector3d neighbor_center(msg->x, msg->y, msg->z);
    formation_centers_[neighbor_name] = neighbor_center;
    formation_center_received_ = true;
    
    ROS_DEBUG("[%s] Received formation center from %s: (%.3f, %.3f, %.3f)",
              uav_name_.c_str(), neighbor_name.c_str(), msg->x, msg->y, msg->z);
}

Eigen::Vector3d UAVStateMachine::calculateAverageFormationCenter() {
    if (formation_centers_.empty()) {
        ROS_WARN("[%s] No formation centers available for averaging", uav_name_.c_str());
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d avg_center = Eigen::Vector3d::Zero();
    for (const auto& center_pair : formation_centers_) {
        avg_center += center_pair.second;
    }
    avg_center /= formation_centers_.size();
    ROS_INFO("[%s] Calculated average formation center from %d neighbors: (%.3f, %.3f, %.3f)",
             uav_name_.c_str(), (int)formation_centers_.size(),
             avg_center.x(), avg_center.y(), avg_center.z());
    return avg_center;
}

Eigen::Vector3d UAVStateMachine::calculateEmergencyRecoveryPosition() {
    Eigen::Vector3d recovery_pos;
    if (topology_manager_) {
        updateDynamicFormationOffsets(target_formation_type_);
        ROS_INFO("[%s] Updated formation offsets for recovery, online count: %d", 
                 uav_name_.c_str(), topology_manager_->getOnlineCount());
    }
    if (!formation_centers_.empty()) {
        Eigen::Vector3d avg_center = calculateAverageFormationCenter();
        recovery_pos = Eigen::Vector3d(
            avg_center.x() + offset_x_,
            avg_center.y() + offset_y_,
            avg_center.z()
        );
        start_position_ = recovery_pos;
        ROS_INFO("[%s] Using average formation center for recovery: (%.3f, %.3f, %.3f)", 
                 uav_name_.c_str(), recovery_pos.x(), recovery_pos.y(), recovery_pos.z());
    }
    return recovery_pos;
}