// uav_agent.cpp
#include "offboard_pkg/uav_agent.h"

UAVAgent::UAVAgent(const ros::NodeHandle& nh, const std::string& uav_name, double offset_x, double offset_y, double local_offset_x_, double local_offset_y_)
    : uav_name_(uav_name), offset_x_(offset_x), offset_y_(offset_y), local_offset_x_(offset_x), local_offset_y_(offset_y), nh_ns_("/" + uav_name), nh_obs(nh), mpc_controller_(nullptr)
{
    state_sub_ = nh_ns_.subscribe("mavros/state", 10, &UAVAgent::stateCallback, this);
    local_pos_pub_ = nh_ns_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_pose_sub_ = nh_ns_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &UAVAgent::localPoseCallback, this);
    arming_client_ = nh_ns_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_ns_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    velocity_sub_ = nh_ns_.subscribe("mavros/local_position/velocity_local", 10, &UAVAgent::velocityCallback, this);
    
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
    last_request_ = ros::Time::now();

    agent_state_pub_ = nh_ns_.advertise<geometry_msgs::PoseStamped>("/" + uav_name_ + "/agent_state", 10);
    vel_pub_ = nh_ns_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    attitude_target_pub_ = nh_ns_.advertise<mavros_msgs::AttitudeTarget>("/" + uav_name_ + "/mavros/setpoint_raw/attitude", 10);
    attitude_cmd_pub_ = nh_ns_.advertise<mavros_msgs::AttitudeTarget>("/" + uav_name_ + "/mavros/setpoint_raw/attitude", 10);
    offset_pub_ = nh_ns_.advertise<geometry_msgs::Point>("/" + uav_name_ + "/offset", 10);
    error_pub_ = nh_ns_.advertise<geometry_msgs::Vector3>("/" + uav_name_ + "/error_pos", 10);
    desired_pos_pub_ = nh_ns_.advertise<geometry_msgs::PointStamped>("desired_position", 1);
    desired_path_pub_ = nh_ns_.advertise<nav_msgs::Path>("desired_path", 1);
    agent_path_pub_ = nh_ns_.advertise<nav_msgs::Path>("/" + uav_name_ + "/agent_path", 10);
    obstacle_marker_pub_ = nh_obs.advertise<visualization_msgs::Marker>("obstacle_markers", 10);

    param_get_client = nh_ns_.serviceClient<mavros_msgs::ParamGet>("/" + uav_name_ + "/mavros/param/get");
}

void UAVAgent::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

bool UAVAgent::armAndSetMode() {
    bool success = true;

    // 只有在 mode 不是 OFFBOARD 或未解锁时才操作
    if (current_state_.mode != "OFFBOARD") {
        if (set_mode_client_.call(offb_set_mode_)) {
            if (offb_set_mode_.response.mode_sent) {
                ROS_INFO("[%s] Offboard mode enabled", uav_name_.c_str());
            } else {
                ROS_ERROR("[%s] Failed to set Offboard mode!", uav_name_.c_str());
                success = false;
            }
        } else {
            ROS_ERROR("[%s] Service call failed: set_mode", uav_name_.c_str());
            success = false;
        }
    }

    if (!current_state_.armed) {
        if (arming_client_.call(arm_cmd_)) {
            if (arm_cmd_.response.success) {
                ROS_INFO("[%s] Vehicle armed", uav_name_.c_str());
            } else {
                ROS_ERROR("[%s] Failed to arm!", uav_name_.c_str());
                success = false;
            }
        } else {
            ROS_ERROR("[%s] Service call failed: arming", uav_name_.c_str());
            success = false;
        }
    }

    return success;
}

void UAVAgent::publishSetVelocity() {
    vel_pub_.publish(vel_msg_);
}

void UAVAgent::loadObstaclesFromParamServer() {
    // 清空现有障碍物
    obstacle_poses_.clear();
    
    // 获取 ROS 参数
    XmlRpc::XmlRpcValue obstacles_param;
    if (ros::param::get("obstacles", obstacles_param)) {
        // ROS_INFO("[%s] Loading obstacles from parameter server.", uav_name_.c_str());
        double t = ros::Time::now().toSec() - traj_start_time_;
        // 检查参数类型是否为数组
        if (obstacles_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            // 遍历障碍物列表并提取数据
            for (int i = 0; i < obstacles_param.size(); ++i) {
                XmlRpc::XmlRpcValue obstacle = obstacles_param[i];
                // 检查是否为结构体类型
                if (obstacle.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    Obstacle obs;
                    // 检查必要字段是否存在
                    if (obstacle.hasMember("name") && 
                        (obstacle.hasMember("position") || (obstacle.hasMember("x") && obstacle.hasMember("y") && obstacle.hasMember("z"))) && 
                        obstacle.hasMember("radius")) {
                        
                        obs.name = static_cast<std::string>(obstacle["name"]);

                        // 检查是否为动态障碍物
                        bool is_dynamic = false;
                        if (obstacle.hasMember("is_dynamic")) {
                            is_dynamic = static_cast<bool>(obstacle["is_dynamic"]);
                            obs.is_dynamic = is_dynamic;
                        }
                        
                        // 处理位置信息
                        if (obstacle.hasMember("position")) {
                            // 位置作为数组格式
                            XmlRpc::XmlRpcValue position = obstacle["position"];
                            // 静态障碍物赋值
                            if (position.getType() == XmlRpc::XmlRpcValue::TypeArray && position.size() == 3 && !obs.is_dynamic) {
                                obs.position = Eigen::Vector3d(
                                    static_cast<double>(position[0]),
                                    static_cast<double>(position[1]),
                                    static_cast<double>(position[2])
                                );
                            }
                            // 动态障碍物赋值
                            else if(position.getType() == XmlRpc::XmlRpcValue::TypeArray && position.size() == 3 && obs.is_dynamic){
                                XmlRpc::XmlRpcValue velocity = obstacle["velocity"];
                                if(velocity.getType() == XmlRpc::XmlRpcValue::TypeArray && velocity.size() == 3){
                                    obs.velocity = Eigen::Vector3d(
                                        static_cast<double>(velocity[0]),
                                        static_cast<double>(velocity[1]),
                                        static_cast<double>(velocity[2])
                                    );
                                    obs.position = Eigen::Vector3d(
                                        static_cast<double>(position[0]) + obs.velocity.x() * t,
                                        static_cast<double>(position[1]) + obs.velocity.y() * t,
                                        static_cast<double>(position[2]) + obs.velocity.z() * t
                                    );
                                }
                            }
                        } 
                        // 处理半径信息
                        obs.radius = static_cast<double>(obstacle["radius"]);    
                        obstacle_poses_.push_back(obs);
                        // ROS_INFO("[%s] Loaded obstacle: %s at (%.2f, %.2f, %.2f) with radius %.2f",
                        //         uav_name_.c_str(), obs.name.c_str(), 
                        //         obs.position.x(), obs.position.y(), obs.position.z(), 
                        //         obs.radius);
                    } else {
                        ROS_WARN("[%s] Obstacle at index %d is missing required fields (name, position/xyz, radius)", 
                                uav_name_.c_str(), i);
                    }
                }
            }
        }
    } else {
        ROS_WARN("[%s] No obstacles parameter found on the parameter server.", uav_name_.c_str());
    }
}

Eigen::Vector3d UAVAgent::calculateObstacleAvoidanceForce() {
    // 三维
    // Eigen::Vector3d total_force(0, 0, 0);
    // Eigen::Vector3d current_pos(
    //     pose_.pose.position.x, 
    //     pose_.pose.position.y, 
    //     pose_.pose.position.z
    // );
    
    // for (const auto& obstacle : obstacle_poses_) {
    //     Eigen::Vector3d obs_pos = obstacle.position;
    //     double safe_distance = obstacle.radius + 0.3; // 安全距离（障碍物半径+缓冲）
    //     Eigen::Vector3d diff = current_pos - obs_pos;
    //     double distance = diff.norm();
        
    //     if (distance < safe_distance) {
    //         // 斥力计算（与距离成反比）
    //         double repulsive_gain = 10; // 斥力系数，可调
    //         double force_magnitude = repulsive_gain * (1.0 / (distance - obstacle.radius + 1e-3));
    //         total_force += force_magnitude * diff.normalized();
    //     }
    // }
    // ROS_WARN("[%s], total_force = (%.3f, %.3f, %.3f)", uav_name_.c_str(), total_force.x(), total_force.y(), total_force.z());
    // return total_force;
    Eigen::Vector3d total_force(0, 0, 0);
    Eigen::Vector3d current_pos(
        pose_.pose.position.x, 
        pose_.pose.position.y, 
        pose_.pose.position.z
    );
    
    for (const auto& obstacle : obstacle_poses_) {
        // 只考虑XY平面上的距离
        double dx = current_pos.x() - obstacle.position.x();
        double dy = current_pos.y() - obstacle.position.y();
        double distance_xy = std::sqrt(dx*dx + dy*dy);
        
        double safe_distance = obstacle.radius + 0.3; // 安全距离（障碍物半径+缓冲）
        
        if (distance_xy < safe_distance) {
            // 斥力计算（与距离成反比）- 只在XY平面上
            double repulsive_gain = 10; // 斥力系数，可调
            double force_magnitude = repulsive_gain * (1.0 / (distance_xy - obstacle.radius + 1e-3));
            
            // 斥力方向是从障碍物指向当前位置，但只在XY平面上
            Eigen::Vector3d force_dir(dx, dy, 0);
            force_dir.normalize();
            
            total_force += force_magnitude * force_dir;
        }
    }
    
    // 确保Z方向没有避障力
    total_force.z() = 0;
    
    // ROS_WARN("[%s], total_force = (%.3f, %.3f, %.3f)", uav_name_.c_str(), 
    //          total_force.x(), total_force.y(), total_force.z());
    return total_force;
}

// 获取障碍物列表
std::vector<Obstacle> UAVAgent::getObstacles() const {
    return obstacle_poses_;
}

// void UAVAgent::updateTrajectory(double dt) {
//     if (!planner_) {
//         ROS_ERROR("[%s] Planner is null!", uav_name_.c_str());
//         return;
//     }
    
//     // 添加调试信息
//     ROS_INFO_THROTTLE(5.0, "[%s] Planner trajectory_generated_: %s", 
//                       uav_name_.c_str(), 
//                       planner_->isTrajectoryGenerated() ? "true" : "false");
                      
//     double desired_x = 0.0, desired_y = 0.0, desired_z = 5.0;
//     int count = 0;

//     Eigen::Vector3d velocity_consensus(0, 0, 0);
//     // Eigen::Vector2d velocity_consensus(0, 0);
//     int vel_count = 0;

//     // === leader 自主生成轨迹 ===
//     //圆形编队
//         // double radius = 2.0;
//         // double period = 20.0; // 转一圈所需时间，单位：秒
//         // double omega = 2 * M_PI / period;

//         // desired_x = offset_x_ + radius * cos(omega * t);
//         // desired_y = offset_y_ + radius * sin(omega * t);
//     //************************************************************************ */
//     // if (uav_name_ == "uav0") {
//     //     static double start_time = ros::Time::now().toSec();
//     //     double t = ros::Time::now().toSec() - start_time;
//     //     double v_set = 0.1;
    
//     //     // 直线匀速目标
//     //     desired_x = offset_x_ + v_set * t;
//     //     desired_y = offset_y_;
//     //     desired_z = 5.0;
    
//     //     // 期望速度
//     //     double desired_vx = v_set;
//     //     double desired_vy = 0.0;
//     //     double desired_vz = 0.0;
    
//     //     // 误差计算
//     //     double dx = desired_x - pose_.pose.position.x;
//     //     double dy = desired_y - pose_.pose.position.y;
//     //     double dz = desired_z - pose_.pose.position.z;
    
//     //     double dvx = desired_vx - current_velocity_.twist.linear.x;
//     //     double dvy = desired_vy - current_velocity_.twist.linear.y;
//     //     double dvz = desired_vz - current_velocity_.twist.linear.z;
    
//     //     // 发布误差
//     //     geometry_msgs::Vector3 error_msg;
//     //     error_msg.x = dx;
//     //     error_msg.y = dy;
//     //     error_msg.z = dz;
//     //     error_pub_.publish(error_msg);
    
//     //     // PD控制器
//     //     double Kp = 1.2;
//     //     double Kd = 0.8;
//     //     double Kp_z = 1.0;
//     //     double Kd_z = 0.5;
    
//     //     double ux = Kp * dx + Kd * dvx;
//     //     double uy = Kp * dy + Kd * dvy;
//     //     double uz = Kp_z * dz + Kd_z * dvz;
    
//     //     vel_msg_.twist.linear.x = ux;
//     //     vel_msg_.twist.linear.y = uy;
//     //     vel_msg_.twist.linear.z = uz;
//     //     return;
//     // }
//     //************************************************************************ */
//     // 计算避障斥力
//     Eigen::Vector3d avoidance_force = calculateObstacleAvoidanceForce();
//     if (uav_name_ == "uav0" && planner_) {
//         double t = ros::Time::now().toSec() - traj_start_time_;
//         ROS_WARN("TIME_NOW = %lf, traj_start_time_ = %lf", ros::Time::now().toSec(), traj_start_time_);
//         ROS_WARN("[%s] t = %.3f", uav_name_.c_str(), t);
    
//         Eigen::Vector3d desired_pos, desired_vel;
    
//         // 获取轨迹的终点位置
//         Eigen::Vector3d end_pos = planner_->getEndPosition(); // 获取轨迹终点
    
//         if (!reached_goal && planner_->sampleTrajectory(t, desired_pos, desired_vel)) {
//             Eigen::Vector3d current_pos(pose_.pose.position.x,
//                                         pose_.pose.position.y,
//                                         pose_.pose.position.z);
//             // 计算与终点位置的误差
//             Eigen::Vector3d diff = end_pos - current_pos;
//             double distance_to_goal = diff.norm();  // 使用与终点的距离
    
//             double velocity_threshold = 0.5;
    
//             // 判断是否到达目标
//             if (distance_to_goal < 0.2 &&
//                 std::abs(current_velocity_.twist.linear.x) < velocity_threshold &&
//                 std::abs(current_velocity_.twist.linear.y) < velocity_threshold &&
//                 std::abs(current_velocity_.twist.linear.z) < velocity_threshold) {
//                 // 标记为到达目标点
//                 reached_goal = true;
//             }
//             ROS_WARN("[%s] distance_to_goal = %.3f", uav_name_.c_str(), distance_to_goal);
//             ROS_WARN("[%s] desired_pos = (%.3f, %.3f, %.3f)", uav_name_.c_str(), desired_pos.x(), desired_pos.y(), desired_pos.z());
//             geometry_msgs::PointStamped point_msg;
//             point_msg.header.frame_id = "map";
//             point_msg.header.stamp = ros::Time::now();
//             point_msg.point.x = desired_pos.x();
//             point_msg.point.y = desired_pos.y();
//             point_msg.point.z = desired_pos.z();
//             desired_pos_pub_.publish(point_msg);
//             // 发布可视化轨迹
//             geometry_msgs::PoseStamped pose;
//             pose.header = point_msg.header; // 使用相同的时间戳和坐标系
//             pose.pose.position.x = desired_pos.x();
//             pose.pose.position.y = desired_pos.y();
//             pose.pose.position.z = desired_pos.z();
//             pose.pose.orientation.w = 1.0; // 单位四元数，表示无旋转
//             desired_path.header = point_msg.header; // 更新Path的Header
//             desired_path.poses.push_back(pose); // 将当前点加入轨迹
//             desired_path_pub_.publish(desired_path); // 发布轨迹
//         }
//         // 控制逻辑：即使已到达，也继续控制 UAV 稳定在终点位置
//         Eigen::Vector3d target_pos = reached_goal ? end_pos : desired_pos;
//         Eigen::Vector3d target_vel = reached_goal ? Eigen::Vector3d::Zero() : desired_vel;
    
//         Eigen::Vector3d current_pos(pose_.pose.position.x,
//                                     pose_.pose.position.y,
//                                     pose_.pose.position.z);
//         Eigen::Vector3d current_vel(current_velocity_.twist.linear.x,
//                                     current_velocity_.twist.linear.y,
//                                     current_velocity_.twist.linear.z);
    
//         Eigen::Vector3d pos_error = target_pos - current_pos;
//         Eigen::Vector3d vel_error = target_vel - current_vel;
        
//         // 用APF控制做
//         // vel_msg_.twist.linear.x = 1.0 * pos_error.x() + 0.5 * vel_error.x() + avoidance_force.x();
//         // vel_msg_.twist.linear.y = 1.0 * pos_error.y() + 0.5 * vel_error.y() + avoidance_force.y();
//         // vel_msg_.twist.linear.z = 1.0 * pos_error.z() + 0.5 * vel_error.z() + avoidance_force.z();
        
//         // 用规划做
//         vel_msg_.twist.linear.x = 1.0 * pos_error.x() + 0.5 * vel_error.x();
//         vel_msg_.twist.linear.y = 1.0 * pos_error.y() + 0.5 * vel_error.y();
//         vel_msg_.twist.linear.z = 1.0 * pos_error.z() + 0.5 * vel_error.z();
//         ROS_WARN("[%s], avoidance_force = (%.3f, %.3f, %.3f)", uav_name_.c_str(), avoidance_force.x(), avoidance_force.y(), avoidance_force.z());
//         ROS_WARN("[%s] end_pos = (%.3f, %.3f, %.3f)", uav_name_.c_str(), end_pos.x(), end_pos.y(), end_pos.z());
//         return;
//     }    
//     /************************************************************************ */
//     //=== follower 推算目标位置 ===
//     // for (const auto& kv : neighbor_poses_) {
//     //     const auto& neighbor_name = kv.first;
//     //     const auto& neighbor_pose = kv.second.pose.position;

//     //     if (neighbor_offsets_.find(neighbor_name) == neighbor_offsets_.end()) continue;
//     //     const auto& neighbor_offset = neighbor_offsets_[neighbor_name];

//     //     double delta_x = offset_x_ - neighbor_offset.x;
//     //     double delta_y = offset_y_ - neighbor_offset.y;

//     //     double target_x = neighbor_pose.x + delta_x;
//     //     double target_y = neighbor_pose.y + delta_y;

//     //     desired_x += target_x;
//     //     desired_y += target_y;
//     //     count++;

//     //     if (neighbor_twists_.find(neighbor_name) != neighbor_twists_.end()) {
//     //         const auto& vel = neighbor_twists_[neighbor_name].twist.linear;
//     //         velocity_consensus += Eigen::Vector2d(vel.x, vel.y);
//     //         vel_count++;
//     //     }
//     // }

//     // if (count > 0) {
//     //     desired_x /= count;
//     //     desired_y /= count;

//     //     double Kp = 1.5;
//     //     double Kd = 1.0;
//     //     double Kp_z = 1.2;
//     //     double Kd_z = 0.4;

//     //     double e_x = desired_x - pose_.pose.position.x;
//     //     double e_y = desired_y - pose_.pose.position.y;
//     //     double e_z = desired_z - pose_.pose.position.z;

//     //     double e_vx = 0.0 - current_velocity_.twist.linear.x;
//     //     double e_vy = 0.0 - current_velocity_.twist.linear.y;
//     //     double e_vz = 0.0 - current_velocity_.twist.linear.z;

//     //     if (vel_count > 0) {
//     //         velocity_consensus /= vel_count;
//     //         e_vx = velocity_consensus.x() - current_velocity_.twist.linear.x;
//     //         e_vy = velocity_consensus.y() - current_velocity_.twist.linear.y;
//     //     }

//     //     double u_x = Kp * e_x + Kd * e_vx;
//     //     double u_y = Kp * e_y + Kd * e_vy;
//     //     double u_z = Kp_z * e_z + Kd_z * e_vz;

//     //     vel_msg_.twist.linear.x = u_x;
//     //     vel_msg_.twist.linear.y = u_y;
//     //     vel_msg_.twist.linear.z = u_z;

//     //     // === 打印误差 ===
//     //     ROS_INFO("[%s] e_pos=(%.3f, %.3f, %.3f)", 
//     //         uav_name_.c_str(), e_x, e_y, e_z);

//     //     // 发布误差
//     //     geometry_msgs::Vector3 error_msg;
//     //     error_msg.x = e_x;
//     //     error_msg.y = e_y;
//     //     error_msg.z = e_z;
//     //     error_pub_.publish(error_msg);            
//     // } 
//     /*********************************************************************************** */
//     // 跟随者UAV的轨迹执行与避障
//     if (uav_name_ != "uav0" && planner_) {
//         // 检测前方障碍物
//         bool obstacle_ahead = detectObstacleAhead(4.0, 0.8);
        
//         // 如果检测到障碍物且当前不在避障模式
//         if (obstacle_ahead && !in_avoidance_mode_) {
//             ROS_WARN("[%s] Obstacle detected! Starting local replanning.", uav_name_.c_str());
//             in_avoidance_mode_ = true;
            
//             // 保存原始轨迹的起始时间
//             original_traj_start_time_ = traj_start_time_;
            
//             // 保存当前位置作为重规划起点
//             Eigen::Vector3d current_pos(pose_.pose.position.x, 
//                                        pose_.pose.position.y, 
//                                        pose_.pose.position.z);
//             // 保存当前速度
//             Eigen::Vector3d current_vel(current_velocity_.twist.linear.x,
//             current_velocity_.twist.linear.y,
//             current_velocity_.twist.linear.z);
            
//             // 获取原轨迹终点
//             Eigen::Vector3d end_pos = planner_->getEndPosition();
            
//             // 计算剩余时间
//             double elapsed_time = ros::Time::now().toSec() - traj_start_time_;
//             double remaining_time = planner_->getDuration() - elapsed_time;
            
//             // 确保剩余时间合理
//             if (remaining_time < 1.0) remaining_time = 1.0;
            
//             // 创建新的规划器进行局部重规划
//             CustomTrajectoryPlanner* local_planner = new CustomTrajectoryPlanner();
//             local_planner->setObstacles(obstacle_poses_);
            
//             // 执行考虑当前速度的A*+Minisnap重规划
//             bool success = local_planner->generatePathWithInitialVelocity(
//             current_pos, end_pos, current_vel, remaining_time);
            
//             // // 执行A*+Minisnap重规划
//             // bool success = local_planner->generatePathWithAStarAndMinisnap(
//             //     current_pos, end_pos, remaining_time, 0.2);
                
//             if (success) {
//                 // 保存原来的规划器
//                 original_planner_ = planner_;
                
//                 // 使用新的规划器
//                 planner_ = local_planner;
                
//                 // 重置轨迹时间
//                 traj_start_time_ = ros::Time::now().toSec();
//                 reached_goal = false;
                
//                 ROS_WARN("[%s] Local replanning successful, executing avoidance trajectory", 
//                          uav_name_.c_str());
//             } else {
//                 ROS_ERROR("[%s] Local replanning failed! Continuing with original trajectory", 
//                           uav_name_.c_str());
//                 delete local_planner;
//                 in_avoidance_mode_ = false;
//             }
//         }
        
//         // 如果在避障模式且已经脱离障碍区域
//         if (in_avoidance_mode_ && !detectObstacleAhead(4.0, 0.8)) {
//             // 检查是否已经安全一段时间
//             if (!safety_timer_started_) {
//                 safety_start_time_ = ros::Time::now().toSec();
//                 safety_timer_started_ = true;
//                 ROS_INFO("[%s] Potentially clear of obstacles, monitoring for %.1f seconds", 
//                          uav_name_.c_str(), 1.0);
//             } else if (ros::Time::now().toSec() - safety_start_time_ > 1.0) {
//                 // 安全1.0s后，直接切换回原始轨迹
//                 ROS_INFO("[%s] Obstacle area cleared, switching back to original trajectory", 
//                          uav_name_.c_str());
                
//                 // 直接切换回原始规划器
//                 delete planner_;
//                 planner_ = original_planner_;
//                 original_planner_ = nullptr;
                
//                 // 恢复原始轨迹的时间
//                 traj_start_time_ = original_traj_start_time_;
                
//                 // 重置状态
//                 in_avoidance_mode_ = false;
//                 safety_timer_started_ = false;
                
//                 ROS_INFO("[%s] Switched back to original trajectory", uav_name_.c_str());
//             }
//         } else if (in_avoidance_mode_ && obstacle_ahead) {
//             // 如果重新检测到障碍物，重置安全计时器
//             if (safety_timer_started_) {
//                 ROS_INFO("[%s] Obstacle still detected, resetting safety timer", uav_name_.c_str());
//                 safety_timer_started_ = false;
//             }
//         }
        
//         // 执行当前活动轨迹
//         double t = ros::Time::now().toSec() - traj_start_time_;
//         Eigen::Vector3d desired_pos, desired_vel;
//         Eigen::Vector3d end_pos = planner_->getEndPosition();
        
//         if (!reached_goal && planner_->sampleTrajectory(t, desired_pos, desired_vel)) {
//             // 现有代码...
//             Eigen::Vector3d current_pos(pose_.pose.position.x,
//                                        pose_.pose.position.y,
//                                        pose_.pose.position.z);
            
//             // 计算与终点位置的误差
//             Eigen::Vector3d diff = end_pos - current_pos;
//             double distance_to_goal = diff.norm();
            
//             // 判断是否到达目标
//             double velocity_threshold = 0.5;
//             if (distance_to_goal < 0.5 &&
//                 std::abs(current_velocity_.twist.linear.x) < velocity_threshold &&
//                 std::abs(current_velocity_.twist.linear.y) < velocity_threshold &&
//                 std::abs(current_velocity_.twist.linear.z) < velocity_threshold) {
//                 reached_goal = true;
//             }
            
//             // 发布期望位置
//             geometry_msgs::PointStamped point_msg;
//             point_msg.header.frame_id = "map";
//             point_msg.header.stamp = ros::Time::now();
//             point_msg.point.x = desired_pos.x();
//             point_msg.point.y = desired_pos.y();
//             point_msg.point.z = desired_pos.z();
//             desired_pos_pub_.publish(point_msg);
        
//             // 发布可视化轨迹
//             geometry_msgs::PoseStamped pose;
//             pose.header = point_msg.header; // 使用相同的时间戳和坐标系
//             pose.pose.position.x = desired_pos.x();
//             pose.pose.position.y = desired_pos.y();
//             pose.pose.position.z = desired_pos.z();
//             pose.pose.orientation.w = 1.0; // 单位四元数，表示无旋转
//             desired_path.header = point_msg.header; // 更新Path的Header
//             desired_path.poses.push_back(pose); // 将当前点加入轨迹
//             desired_path_pub_.publish(desired_path); // 发布轨迹
//         }
        
//         // 控制逻辑
//         Eigen::Vector3d current_pos(pose_.pose.position.x,
//                                    pose_.pose.position.y,
//                                    pose_.pose.position.z);
//         Eigen::Vector3d current_vel(current_velocity_.twist.linear.x,
//                                    current_velocity_.twist.linear.y,
//                                    current_velocity_.twist.linear.z);
                                   
//         Eigen::Vector3d target_pos = reached_goal ? end_pos : desired_pos;
//         Eigen::Vector3d target_vel = reached_goal ? Eigen::Vector3d::Zero() : desired_vel;
        
//         Eigen::Vector3d pos_error = target_pos - current_pos;
//         Eigen::Vector3d vel_error = target_vel - current_vel;
        
//         // 控制输出计算
//         double Kp = 1.0, Kd = 0.5;
//         double Kp_z = 1.0, Kd_z = 0.5;
        
//         vel_msg_.twist.linear.x = Kp * pos_error.x() + Kd * vel_error.x();
//         vel_msg_.twist.linear.y = Kp * pos_error.y() + Kd * vel_error.y();
//         vel_msg_.twist.linear.z = Kp_z * pos_error.z() + Kd_z * vel_error.z();
        
//         // 发布误差
//         geometry_msgs::Vector3 error_msg;
//         error_msg.x = pos_error.x();
//         error_msg.y = pos_error.y();
//         error_msg.z = pos_error.z();
//         error_pub_.publish(error_msg);        
//         return;
//     }
    
//     // 默认情况
//     vel_msg_.twist.linear.x = 0.0;
//     vel_msg_.twist.linear.y = 0.0;
//     vel_msg_.twist.linear.z = 0.0;
// }    

void UAVAgent::setTrajectoryPlanner(CustomTrajectoryPlanner* planner) {
    planner_ = planner;
    traj_start_time_ = ros::Time::now().toSec();
    reached_goal = false;
}

void UAVAgent::publishAgentState() {
    geometry_msgs::PoseStamped msg = pose_;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    agent_state_pub_.publish(msg);

    // 发布可视化轨迹
    agent_path.header = msg.header; // 更新Path的Header
    agent_path.poses.push_back(pose_); // 将当前点加入轨迹
    agent_path_pub_.publish(agent_path); // 发布轨迹
}

void UAVAgent::neighborCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& neighbor_name) {
    ROS_INFO_THROTTLE(0.2, "[%s] Received from [%s]: (%.2f, %.2f)", 
        uav_name_.c_str(), neighbor_name.c_str(), msg->pose.position.x, msg->pose.position.y);
}

void UAVAgent::registerNeighbor(const std::string& neighbor_name) {
    // 使用 boost::bind 绑定回调函数并传入 neighbor_name
    // 订阅position
    ros::NodeHandle nh_ns("/" + neighbor_name);
    neighbor_subs_[neighbor_name] = nh_ns.subscribe<geometry_msgs::PoseStamped>("/" + neighbor_name + "/agent_state", 10,
        boost::bind(&UAVAgent::neighborPoseCallback, this, _1, neighbor_name));
    ROS_INFO("[%s] Registered neighbor: %s", uav_name_.c_str(), neighbor_name.c_str());

    // 订阅 offset
    neighbor_offset_subs_[neighbor_name] = nh_ns.subscribe<geometry_msgs::Point>("/" + neighbor_name + "/offset", 10,
        boost::bind(&UAVAgent::neighborOffsetCallback, this, _1, neighbor_name));

    // 订阅 velocity
    neighbor_twist_subs_[neighbor_name] = nh_ns.subscribe<geometry_msgs::TwistStamped>("/" + neighbor_name + "/mavros/local_position/velocity_local", 10,
        boost::bind(&UAVAgent::neighborTwistCallback, this, _1, neighbor_name));

    neighbor_names_.push_back(neighbor_name);  // 添加这行
}

std::vector<Eigen::Vector3d> UAVAgent::getNeighborPositions() const {
    std::vector<Eigen::Vector3d> positions;
    for (const auto& neighbor_name : neighbor_names_) {
        auto it = neighbor_poses_.find(neighbor_name);
        if (it != neighbor_poses_.end()) {
            positions.emplace_back(
                it->second.pose.position.x,
                it->second.pose.position.y,
                it->second.pose.position.z
            );
        }
    }
    return positions;
}

std::vector<Eigen::Vector3d> UAVAgent::getNeighborVelocities() const {
    std::vector<Eigen::Vector3d> velocities;
    for (const auto& neighbor_name : neighbor_names_) {
        auto it = neighbor_twists_.find(neighbor_name);
        if (it != neighbor_twists_.end()) {
            velocities.emplace_back(
                it->second.twist.linear.x,
                it->second.twist.linear.y,
                it->second.twist.linear.z
            );
        } else {
            velocities.emplace_back(0, 0, 0);  // 默认零速度
        }
    }
    return velocities;
}

void UAVAgent::neighborPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                                    const std::string& neighbor_name) {
    neighbor_poses_[neighbor_name] = *msg;
    // ROS_INFO_THROTTLE(0.2,"[%s] Received pose from neighbor %s: x=%.2f y=%.2f",
    //          uav_name_.c_str(), neighbor_name.c_str(),
    //          msg->pose.position.x, msg->pose.position.y);
}

void UAVAgent::publishOffset() {
    geometry_msgs::Point offset_msg;
    offset_msg.x = offset_x_;
    offset_msg.y = offset_y_;
    offset_msg.z = 0.0;
    offset_pub_.publish(offset_msg);
}

void UAVAgent::neighborOffsetCallback(const geometry_msgs::Point::ConstPtr& msg,
    const std::string& neighbor_name) {
    neighbor_offsets_[neighbor_name] = *msg;
}

void UAVAgent::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_.pose.position.x = msg->pose.position.x + local_offset_x_;
    pose_.pose.position.y = msg->pose.position.y + local_offset_y_;
    pose_.pose.position.z = msg->pose.position.z;
}

void UAVAgent::neighborTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& neighbor_name) {
    neighbor_twists_[neighbor_name] = *msg;
}

void UAVAgent::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_velocity_ = *msg;
}

void UAVAgent::resetTrajectoryTime() {
    traj_start_time_ = ros::Time::now().toSec();
    reached_goal = false;
    ROS_INFO("[%s] Trajectory execution time reset", uav_name_.c_str());

    // 确保轨迹已经生成
    if (planner_ && !planner_->isTrajectoryGenerated()) {
        ROS_WARN("[%s] Resetting trajectory time, but trajectory is not generated yet!", uav_name_.c_str());
    } else {
        ROS_INFO("[%s] Trajectory execution time reset", uav_name_.c_str());
    }
}

// 障碍物检测
bool UAVAgent::detectObstacleAhead(double look_ahead_distance, double safety_radius) {
    Eigen::Vector3d current_pos(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
    Eigen::Vector3d current_vel(current_velocity_.twist.linear.x, 
                                current_velocity_.twist.linear.y, 
                                current_velocity_.twist.linear.z);
    
    // 归一化速度方向
    Eigen::Vector3d direction = current_vel.normalized();

    double step_size = 0.5;
    
    // 检查前方路径上的多个点
    for (double dist = 0.5; dist <= look_ahead_distance; dist += step_size) {
        Eigen::Vector3d check_point = current_pos + direction * dist;
        
        // 检查该点是否与任何障碍物碰撞
        for (const auto& obstacle : obstacle_poses_) {
            double distance = (check_point - obstacle.position).norm() - obstacle.radius;
            if (distance < safety_radius) {
                ROS_WARN("[%s] Obstacle detected ahead at distance %.2f", uav_name_.c_str(), dist);
                return true;
            }
        }
    }
    
    return false;
}

void UAVAgent::publishObstacles() {
    visualization_msgs::Marker marker_array;
    marker_array.header.frame_id = "map"; // 坐标系
    marker_array.header.stamp = ros::Time::now();
    marker_array.ns = "obstacles";
    marker_array.type = visualization_msgs::Marker::CYLINDER; // 圆柱体表示障碍物
    marker_array.action = visualization_msgs::Marker::ADD;
    marker_array.lifetime = ros::Duration(0); // 永久显示

    int id = 0; // 障碍物唯一 ID
    for (const auto& obstacle : obstacle_poses_) {
        visualization_msgs::Marker marker;
        marker.header = marker_array.header;
        marker.ns = marker_array.ns;
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置障碍物位置
        marker.pose.position.x = obstacle.position.x();
        marker.pose.position.y = obstacle.position.y();
        marker.pose.position.z = 2.5; // 圆柱体的中心高度
        marker.pose.orientation.w = 1.0; // 无旋转

        // 设置障碍物大小
        marker.scale.x = obstacle.radius * 2; // 圆柱体直径
        marker.scale.y = obstacle.radius * 2; // 圆柱体直径
        marker.scale.z = 5.0; // 圆柱体高度（可根据需要调整）

        // 设置颜色
        marker.color.r = 1.0; // 红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8; // 透明度

        // 发布单个障碍物
        obstacle_marker_pub_.publish(marker);
    }
}

UAVAgent::MPCControlOutput UAVAgent::calculateMPCControl(
    const Eigen::Vector3d& desired_pos, 
    const Eigen::Vector3d& desired_vel, 
    double current_time,
    const std::vector<Eigen::Vector3d>& neighbor_positions,
    const std::vector<Eigen::Vector3d>& neighbor_velocities) {
    
    MPCControlOutput control_output;
    control_output.success = false;

    // 初始化MPC控制器（如果还没有）
    if (!mpc_controller_) {
        mpc_controller_ = std::make_unique<MPCController>();
        if (!mpc_controller_->initialize()) {
            ROS_ERROR("[%s] Failed to initialize MPC controller", uav_name_.c_str());
            return control_output;
        }
    }

    // 获取当前状态
    Eigen::Vector3d current_pos(pose_.pose.position.x, 
                               pose_.pose.position.y, 
                               pose_.pose.position.z);
    Eigen::Vector3d current_vel(current_velocity_.twist.linear.x,
                               current_velocity_.twist.linear.y,
                               current_velocity_.twist.linear.z);

    // 构建参考轨迹
    int horizon = 15;
    double dt = 0.05;
    std::vector<Eigen::Vector3d> ref_positions, ref_velocities;
    
    Eigen::Vector3d end_pos = planner_->getEndPosition();
    
    for (int k = 0; k < horizon; ++k) {
        double future_time = current_time + k * dt;
        Eigen::Vector3d ref_pos, ref_vel;
        
        if (future_time > planner_->getDuration()) {
            ref_pos = end_pos;
            ref_vel = Eigen::Vector3d::Zero();
        } else {
            planner_->sampleTrajectory(future_time, ref_pos, ref_vel);
        }
        
        ref_positions.push_back(ref_pos);
        ref_velocities.push_back(ref_vel);
    }

    // 获取推力映射参数
    // double thr2acc = getThr2AccFromPX4();
    double thr2acc = 9.81 / 0.7;
    mpc_controller_->updateThr2Acc(thr2acc);

    // 求解MPC
    MPCController::ControlOutput mpc_output = mpc_controller_->solve(
        current_pos, current_vel, ref_positions, ref_velocities, thr2acc, obstacle_poses_, neighbor_positions, neighbor_velocities, observer);
    if (mpc_output.success) {
        // 转换输出格式
        control_output.attitude = eulerToQuaternion(
            mpc_output.euler_angles.x(),  // roll
            mpc_output.euler_angles.y(),  // pitch
            mpc_output.euler_angles.z()   // yaw
        );
        control_output.thrust = mpc_output.thrust;
        control_output.success = true;
        
        ROS_INFO_THROTTLE(1.0, "[%s] MPC: thrust=%.3f, roll=%.1f°, pitch=%.1f°", 
                         uav_name_.c_str(), mpc_output.thrust,
                         mpc_output.euler_angles.x() * 180.0 / M_PI,
                         mpc_output.euler_angles.y() * 180.0 / M_PI);
    } else {
        ROS_WARN("[%s] MPC solve failed, using fallback control", uav_name_.c_str());
        // 可以在这里添加简单的PID控制作为备用
    }

    return control_output;
}

/// 修改后的updateTrajectory函数，适配姿态MPC控制器
void UAVAgent::updateTrajectory(double dt) {
    if (!planner_) {
        ROS_ERROR("[%s] Planner is null!", uav_name_.c_str());
        return;
    }
    
    ROS_INFO_THROTTLE(5.0, "[%s] Planner trajectory_generated_: %s", 
                      uav_name_.c_str(), 
                      planner_->isTrajectoryGenerated() ? "true" : "false");
    
    // 获取当前时间
    double t = ros::Time::now().toSec() - traj_start_time_;
    
    // 采样期望轨迹点
    Eigen::Vector3d desired_pos, desired_vel;
    Eigen::Vector3d end_pos = planner_->getEndPosition();
    
    if (!reached_goal && planner_->sampleTrajectory(t, desired_pos, desired_vel)) {
        // 检查是否到达目标点
        Eigen::Vector3d current_pos(pose_.pose.position.x,
                                   pose_.pose.position.y,
                                   pose_.pose.position.z);
        
        Eigen::Vector3d diff = end_pos - current_pos;
        double distance_to_goal = diff.norm();
        double velocity_threshold = 1.0;
        
        if (distance_to_goal < 0.3 &&
            std::abs(current_velocity_.twist.linear.x) < velocity_threshold &&
            std::abs(current_velocity_.twist.linear.y) < velocity_threshold &&
            std::abs(current_velocity_.twist.linear.z) < velocity_threshold) {
            reached_goal = true;
        }
        
        // 发布期望位置
        geometry_msgs::PointStamped point_msg;
        point_msg.header.frame_id = "map";
        point_msg.header.stamp = ros::Time::now();
        point_msg.point.x = desired_pos.x();
        point_msg.point.y = desired_pos.y();
        point_msg.point.z = desired_pos.z();
        desired_pos_pub_.publish(point_msg);
        
        // 发布可视化轨迹
        geometry_msgs::PoseStamped pose;
        pose.header = point_msg.header;
        pose.pose.position.x = desired_pos.x();
        pose.pose.position.y = desired_pos.y();
        pose.pose.position.z = desired_pos.z();
        pose.pose.orientation.w = 1.0;
        desired_path.header = point_msg.header;
        desired_path.poses.push_back(pose);
        desired_path_pub_.publish(desired_path);
    }
    
    // 确定控制目标
    Eigen::Vector3d target_pos = reached_goal ? end_pos : desired_pos;
    Eigen::Vector3d target_vel = reached_goal ? Eigen::Vector3d::Zero() : desired_vel;
    
    // 获取当前状态
    Eigen::Vector3d current_pos(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
    Eigen::Vector3d current_vel(current_velocity_.twist.linear.x,
                               current_velocity_.twist.linear.y,
                               current_velocity_.twist.linear.z);
    
    std::vector<Eigen::Vector3d> neighbor_positions = getNeighborPositions();
    std::vector<Eigen::Vector3d> neighbor_velocities = getNeighborVelocities();

    // 使用动力学MPC计算期望姿态和推力
    MPCControlOutput mpc_output = calculateMPCControl(target_pos, target_vel, t, neighbor_positions, neighbor_velocities);
    if (mpc_output.success) {
        mavros_msgs::AttitudeTarget attitude_cmd;
        attitude_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        attitude_cmd.body_rate.x = 0.0;
        attitude_cmd.body_rate.y = 0.0;
        attitude_cmd.body_rate.z = 0.0;
        attitude_cmd.header.stamp = ros::Time::now();
        attitude_cmd.orientation = mpc_output.attitude;
        attitude_cmd.thrust = mpc_output.thrust;
        attitude_cmd_pub_.publish(attitude_cmd);
    } 
    
    // 发布误差信息
    Eigen::Vector3d pos_error = target_pos - current_pos;
    geometry_msgs::Vector3 error_msg;
    error_msg.x = pos_error.x();
    error_msg.y = pos_error.y();
    error_msg.z = pos_error.z();
    error_pub_.publish(error_msg);
}

// 欧拉角转四元数的辅助函数
geometry_msgs::Quaternion UAVAgent::eulerToQuaternion(double roll, double pitch, double yaw) {
    geometry_msgs::Quaternion q;
    
    // 计算四元数分量
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

double UAVAgent::getThr2AccFromPX4() {
    
    
    mavros_msgs::ParamGet srv;
    double thr2acc = 16.0;  // 默认值
    
    // 获取关键参数
    // 1. MPC_THR_HOVER - 悬停推力（归一化）
    srv.request.param_id = "MPC_THR_HOVER";
    if (param_get_client.call(srv)) {
        double hover_thrust = srv.response.value.real;
        thr2acc = 9.81 / hover_thrust;
        // ROS_WARN("[%s] 从PX4获取: MPC_THR_HOVER=%.3f, 计算thr2acc=%.3f", 
        //          uav_name_.c_str(), hover_thrust, thr2acc);
        
    }
    return thr2acc;
}

void UAVAgent::updateOffset(double new_offset_x, double new_offset_y) {
    offset_x_ = new_offset_x;
    offset_y_ = new_offset_y;
    
    // 发布新的偏移量
    publishOffset();
}