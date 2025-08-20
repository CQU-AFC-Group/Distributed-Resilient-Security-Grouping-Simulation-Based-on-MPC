// custom_trajectory_planner.cpp
#include <iostream>
#include <cmath>
#include "offboard_pkg/custom_trajectory_planner.h"
#include <Eigen/Dense> 
#include <nav_msgs/Path.h>
/**
 * 构造函数：初始化轨迹规划器
 * 创建一个新的轨迹规划器实例，并初始化所有成员变量
 */
// CustomTrajectoryPlanner::CustomTrajectoryPlanner()
//     : duration_(0.0), trajectory_generated_(false),
//       attractive_gain_(1.0), repulsive_gain_(100.0), 
//       safe_distance_(1.0), goal_threshold_(0.2) {
//     // 初始化x、y、z三个方向的多项式系数向量，每个向量包含6个元素（五次多项式有6个系数）
//     x_coeffs_.resize(6, 0.0);  // 对应 a0, a1, a2, a3, a4, a5
//     y_coeffs_.resize(6, 0.0);
//     z_coeffs_.resize(6, 0.0);
// }

/**
 * 生成轨迹：计算从起点到终点的平滑轨迹
 * @param start_pos 起始位置（3D向量）
 * @param end_pos 目标位置（3D向量）
 * @param duration 轨迹持续时间（秒）
 * @return 是否成功生成轨迹
 */
bool CustomTrajectoryPlanner::generateTrajectory(const Eigen::Vector3d& start_pos, 
    const Eigen::Vector3d& end_pos,
    double duration) {
    // 检查输入参数有效性
    if (duration <= 0.0) {
    ROS_ERROR("Error: Duration must be positive!");
    return false;
    }

    // 保存起点、终点和持续时间
    start_pos_ = start_pos;
    end_pos_ = end_pos;
    duration_ = duration;

    // 分别计算x、y、z三个方向的多项式系数
    calculateQuinticPolynomial(start_pos_.x(), end_pos_.x(), duration_, x_coeffs_);
    calculateQuinticPolynomial(start_pos_.y(), end_pos_.y(), duration_, y_coeffs_);
    calculateQuinticPolynomial(start_pos_.z(), end_pos_.z(), duration_, z_coeffs_);

    // 生成路径点用于可视化
    path_.clear();
    const int num_points = 50;  // 采样点数量

    // 采样轨迹生成路径点
    for (int i = 0; i <= num_points; i++) {
        double t = i * duration_ / num_points;
        Eigen::Vector3d pos, vel;

        if (sampleTrajectory(t, pos, vel)) {
            path_.push_back(pos);
        }
    }

    // 确保路径至少包含起点和终点
    if (path_.empty()) {
        path_.push_back(start_pos_);
        path_.push_back(end_pos_);
        } 
    else if (path_.size() == 1) {
        path_.push_back(end_pos_);
    }

    // 使用Minisnap平滑路径
    bool minisnap_success = minisnap_trajectory_->generateTrajectory(path_, duration);
    
    if (!minisnap_success) {
        std::cerr << "Minisnap trajectory generation failed! Falling back to direct trajectory." << std::endl;
        return generateTrajectory(start_pos, end_pos, duration);
    }

    // 标记轨迹已生成
    trajectory_generated_ = true;

    ROS_INFO("Trajectory generated from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f) with duration %.2fs", 
    start_pos_.x(), start_pos_.y(), start_pos_.z(), 
    end_pos_.x(), end_pos_.y(), end_pos_.z(), duration_);

    ROS_INFO("Generated path with %zu points", path_.size());

    return true;
}



/**
 * 设置APF参数
 * @param attractive_gain 引力增益
 * @param repulsive_gain 斥力增益
 * @param safe_distance 安全距离
 * @param goal_threshold 目标阈值
 */
void CustomTrajectoryPlanner::setAPFParams(double attractive_gain, double repulsive_gain, 
                                         double safe_distance, double goal_threshold) {
    attractive_gain_ = attractive_gain;
    repulsive_gain_ = repulsive_gain;
    safe_distance_ = safe_distance;
    goal_threshold_ = goal_threshold;
}

/**
 * 设置障碍物信息
 * @param obstacles 障碍物列表
 */
void CustomTrajectoryPlanner::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
}

/**
 * 计算引力
 * @param position 当前位置
 * @param goal 目标位置
 * @return 引力向量
 */
Eigen::Vector3d CustomTrajectoryPlanner::calculateAttractiveForce(const Eigen::Vector3d& position, 
                                                               const Eigen::Vector3d& goal) {
    // 计算从当前位置指向目标的向量
    Eigen::Vector3d diff = goal - position;
    // 返回引力（与距离成正比）
    return attractive_gain_ * diff;
}

/**
 * 计算斥力
 * @param position 当前位置
 * @return 斥力向量
 */
Eigen::Vector3d CustomTrajectoryPlanner::calculateRepulsiveForce(const Eigen::Vector3d& position) {
    Eigen::Vector3d total_force(0, 0, 0);
    
    // 遍历所有障碍物，计算斥力
    for (const auto& obstacle : obstacles_) {
        // 计算从障碍物指向当前位置的向量
        Eigen::Vector3d diff = position - obstacle.position;
        // 计算到障碍物表面的距离（考虑障碍物半径）
        double distance = diff.norm() - obstacle.radius;
        
        // 只有当距离小于安全距离时才产生斥力
        if (distance < safe_distance_) {
            // 改进的斥力模型：距离越近，斥力越大
            double force_magnitude = repulsive_gain_ * 
                (1.0 / distance - 1.0 / safe_distance_) * 
                (1.0 / (distance * distance));
                
            // 斥力方向是从障碍物指向当前位置
            total_force += force_magnitude * diff.normalized();
        }
    }
    
    return total_force;
}

/**
 * 计算总力（引力+斥力）
 * @param position 当前位置
 * @param goal 目标位置
 * @return 总力向量
 */
Eigen::Vector3d CustomTrajectoryPlanner::calculateTotalForce(const Eigen::Vector3d& position, 
                                                          const Eigen::Vector3d& goal) {
    return calculateAttractiveForce(position, goal) + calculateRepulsiveForce(position);
}

/**
 * 处理局部最小值问题
 * @param position 当前位置
 * @param goal 目标位置
 * @return 额外的逃离力
 */
Eigen::Vector3d CustomTrajectoryPlanner::escapeLocalMinimum(const Eigen::Vector3d& position, 
    const Eigen::Vector3d& goal) {
    Eigen::Vector3d force = calculateTotalForce(position, goal);

    // 降低触发阈值，更容易检测到局部最小值
    if (force.norm() < 1.5) 
    {  
        // 计算到目标的向量
        Eigen::Vector3d to_goal = goal - position;
        double dist_to_goal = to_goal.norm();

        // 如果距离目标还远，但力很小，可能陷入了局部最小值
        if (dist_to_goal > 1.0) 
        {
            // 找到所有接近的障碍物
            std::vector<std::pair<double, Eigen::Vector3d>> close_obstacles;

            for (const auto& obstacle : obstacles_) 
            {
                Eigen::Vector3d diff = position - obstacle.position;
                double distance = diff.norm() - obstacle.radius;

                if (distance < safe_distance_ * 2.0) 
                {
                    close_obstacles.push_back({distance, diff.normalized()});
                }
            }

            // 如果有多个接近的障碍物
            if (close_obstacles.size() > 1) 
            {
                // 计算逃离方向（避开所有接近的障碍物）
                Eigen::Vector3d escape_dir(0, 0, 0);
                for (const auto& obs : close_obstacles) 
                {
                    escape_dir += obs.second;
                }

                // 确保逃离方向与目标方向尽可能一致
                if (escape_dir.norm() > 0.001) 
                {
                    escape_dir.normalize();
                    Eigen::Vector3d goal_dir = to_goal.normalized();

                    // 计算一个综合方向，既考虑逃离又考虑目标
                    Eigen::Vector3d combined_dir = escape_dir + goal_dir;
                    if (combined_dir.norm() > 0.001) 
                    {
                        combined_dir.normalize();
                        // 返回一个较强的力来帮助逃离
                        return combined_dir * repulsive_gain_;
                    }
                }
            }

            // 如果上面的方法不起作用，尝试随机方向
            Eigen::Vector3d random_dir = Eigen::Vector3d::Random().normalized();
            // 确保随机方向与目标方向夹角不超过90度
            if (random_dir.dot(to_goal.normalized()) < 0) 
            {
                random_dir = -random_dir;
            }
            return random_dir * repulsive_gain_;
        }
    }
    return Eigen::Vector3d(0, 0, 0);
}



/**
 * 使用APF方法生成避障路径
 * @param start_pos 起始位置
 * @param end_pos 目标位置
 * @param duration 轨迹持续时间
 * @param step_size 路径点间距
 * @return 是否成功生成路径
 */
bool CustomTrajectoryPlanner::generatePathWithAPF(const Eigen::Vector3d& start_pos, 
                                                const Eigen::Vector3d& end_pos,
                                                double duration,
                                                double step_size) {
    // 三维
    // // 检查是否有障碍物信息
    // if (obstacles_.empty()) {
    //     std::cout << "No obstacles detected, using direct trajectory." << std::endl;
    //     return generateTrajectory(start_pos, end_pos, duration);
    // }
                                                  
    // // 清空之前的路径
    // path_.clear();
    // path_.push_back(start_pos);
    
    // Eigen::Vector3d current_pos = start_pos;
    // int max_iterations = 50000;  // 最大迭代次数
    // int stall_count = 0;         // 停滞计数
    // double min_progress = 0.03;  // 最小前进距离
    
    // for (int i = 0; i < max_iterations; ++i) {
    //     // 计算当前位置的势场力
    //     Eigen::Vector3d force = calculateTotalForce(current_pos, end_pos);
        
    //     // 检查是否陷入局部最小值
    //     if (force.norm() < 1.5) {
    //         Eigen::Vector3d escape_force = escapeLocalMinimum(current_pos, end_pos);
    //         if (escape_force.norm() > 0) {
    //             force = escape_force;
    //         }
    //     }
        
    //     // 归一化并按步长移动
    //     if (force.norm() > 1e-6) {  // 避免除以零
    //         Eigen::Vector3d step = force.normalized() * step_size;
    //         Eigen::Vector3d new_pos = current_pos + step;
            
    //         // 检查是否有足够的进展
    //         double progress = (new_pos - current_pos).norm();
    //         if (progress < min_progress) {
    //             stall_count++;
    //             if (stall_count > 10) {  // 如果连续进展不足，考虑添加随机扰动
    //                 Eigen::Vector3d random_step = Eigen::Vector3d::Random() * step_size;
    //                 new_pos = current_pos + random_step;
    //                 stall_count = 0;
    //             }
    //         } else {
    //             stall_count = 0;
    //         }
            
    //         current_pos = new_pos;
    //         path_.push_back(current_pos);
    //     }
        
    //     // 检查是否到达目标
    //     if ((current_pos - end_pos).norm() < goal_threshold_) {
    //         path_.push_back(end_pos);  // 确保最后一点是精确的目标点
    //         break;
    //     }
    // }
    
    // // 如果路径太短或没有到达目标，返回失败
    // if (path_.size() < 2 || (path_.back() - end_pos).norm() > goal_threshold_) {
    //     std::cerr << "Failed to generate path with APF: ";
    //     if (path_.size() < 2) {
    //         std::cerr << "Path too short." << std::endl;
    //     } else {
    //         std::cerr << "Did not reach goal. Distance: " 
    //                   << (path_.back() - end_pos).norm() << std::endl;
    //     }
    //     return false;
    // }
    
    // // 平滑路径
    // path_ = smoothPath(path_);
    
    // // 根据平滑后的路径生成轨迹
    // return fitPolynomialTrajectory(path_, duration);

    //二维
     // 检查是否有障碍物信息
     if (obstacles_.empty()) {
        std::cout << "No obstacles detected, using direct trajectory." << std::endl;
        return generateTrajectory(start_pos, end_pos, duration);
    }
                                                  
    // 清空之前的路径
    path_.clear();
    path_.push_back(start_pos);
    
    Eigen::Vector3d current_pos = start_pos;
    int max_iterations = 50000;  // 最大迭代次数
    int stall_count = 0;         // 停滞计数
    double min_progress = 0.03;  // 最小前进距离
    
    // 保存固定的Z高度
    double fixed_z = start_pos.z();
    
    for (int i = 0; i < max_iterations; ++i) {
        // 计算当前位置的势场力 - 只在XY平面
        Eigen::Vector3d force = calculateTotalForce(current_pos, end_pos);
        force.z() = 0;  // 确保Z方向没有力
        
        // 检查是否陷入局部最小值
        if (force.norm() < 1.5) {
            Eigen::Vector3d escape_force = escapeLocalMinimum(current_pos, end_pos);
            escape_force.z() = 0;  // 确保Z方向没有力
            if (escape_force.norm() > 0) {
                force = escape_force;
            }
        }
        
        // 归一化并按步长移动
        if (force.norm() > 1e-6) {  // 避免除以零
            Eigen::Vector3d step = force.normalized() * step_size;
            step.z() = 0;  // 确保Z方向不移动
            
            Eigen::Vector3d new_pos = current_pos + step;
            new_pos.z() = fixed_z;  // 保持Z高度不变
            
            // 检查是否有足够的进展 - 只考虑XY平面
            double progress = std::sqrt(
                std::pow(new_pos.x() - current_pos.x(), 2) + 
                std::pow(new_pos.y() - current_pos.y(), 2));
                
            if (progress < min_progress) {
                stall_count++;
                if (stall_count > 10) {  // 如果连续进展不足，考虑添加随机扰动
                    Eigen::Vector3d random_step = Eigen::Vector3d::Random() * step_size;
                    random_step.z() = 0;  // 确保Z方向不添加扰动
                    new_pos = current_pos + random_step;
                    new_pos.z() = fixed_z;  // 保持Z高度不变
                    stall_count = 0;
                }
            } else {
                stall_count = 0;
            }
            
            current_pos = new_pos;
            path_.push_back(current_pos);
        }
        
        // 检查是否到达目标 - 只考虑XY平面距离
        double dist_to_goal = std::sqrt(
            std::pow(current_pos.x() - end_pos.x(), 2) + 
            std::pow(current_pos.y() - end_pos.y(), 2));
            
        if (dist_to_goal < goal_threshold_) {
            // 确保最后一点是精确的目标点，但保持Z高度不变
            Eigen::Vector3d final_pos = end_pos;
            final_pos.z() = fixed_z;
            path_.push_back(final_pos);
            break;
        }
    }
    
    // 如果路径太短或没有到达目标，返回失败
    double dist_to_goal = std::sqrt(
        std::pow(path_.back().x() - end_pos.x(), 2) + 
        std::pow(path_.back().y() - end_pos.y(), 2));
        
    if (path_.size() < 2 || dist_to_goal > goal_threshold_) {
        std::cerr << "Failed to generate path with APF: ";
        if (path_.size() < 2) {
            std::cerr << "Path too short." << std::endl;
        } else {
            std::cerr << "Did not reach goal. Distance: " << dist_to_goal << std::endl;
        }
        return false;
    }
    
    // 平滑路径
    path_ = smoothPath(path_);
    
    // 根据平滑后的路径生成轨迹
    return fitPolynomialTrajectory(path_, duration);
}

/**
 * 平滑路径点
 * @param raw_path 原始路径点
 * @return 平滑后的路径点
 */
std::vector<Eigen::Vector3d> CustomTrajectoryPlanner::smoothPath(const std::vector<Eigen::Vector3d>& raw_path) {
    if (raw_path.size() <= 2) return raw_path;  // 如果路径点太少，不需要平滑
    
    std::vector<Eigen::Vector3d> smoothed_path;
    
    // 保留起点
    smoothed_path.push_back(raw_path.front());
    
    // 使用移动平均平滑中间点
    int window_size = 5;
    for (size_t i = 1; i < raw_path.size() - 1; ++i) {
        Eigen::Vector3d avg(0, 0, 0);
        int count = 0;
        
        for (int j = -window_size/2; j <= window_size/2; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < raw_path.size()) {
                avg += raw_path[idx];
                count++;
            }
        }
        
        if (count > 0) {
            smoothed_path.push_back(avg / count);
        }
    }
    
    // 保留终点
    smoothed_path.push_back(raw_path.back());
    
    // 进一步减少路径点数量（可选）
    if (smoothed_path.size() > 50) {  // 如果路径点太多
        std::vector<Eigen::Vector3d> decimated_path;
        decimated_path.push_back(smoothed_path.front());  // 保留起点
        
        // 每隔几个点取一个点
        int step = smoothed_path.size() / 50;
        for (size_t i = step; i < smoothed_path.size() - 1; i += step) {
            decimated_path.push_back(smoothed_path[i]);
        }
        
        decimated_path.push_back(smoothed_path.back());  // 保留终点
        return decimated_path;
    }
    
    return smoothed_path;
}

/**
 * 将路径点拟合为多项式轨迹
 * @param path 路径点
 * @param duration 轨迹持续时间
 * @return 是否成功拟合
 */
bool CustomTrajectoryPlanner::fitPolynomialTrajectory(const std::vector<Eigen::Vector3d>& path, double duration) {
    if (path.size() < 2) {
        std::cerr << "Path too short to fit trajectory." << std::endl;
        return false;
    }
    
    // 保存起点和终点
    start_pos_ = path.front();
    end_pos_ = path.back();
    duration_ = duration;
    
    // 如果路径点较少，直接使用五次多项式拟合
    if (path.size() <= 3) {
        calculateQuinticPolynomial(start_pos_.x(), end_pos_.x(), duration_, x_coeffs_);
        calculateQuinticPolynomial(start_pos_.y(), end_pos_.y(), duration_, y_coeffs_);
        calculateQuinticPolynomial(start_pos_.z(), end_pos_.z(), duration_, z_coeffs_);
    } else {
        // 使用分段多项式拟合路径
        fitSegmentedTrajectory(path, duration);
    }
    
    trajectory_generated_ = true;
    return true;
}

bool CustomTrajectoryPlanner::fitSegmentedTrajectory(const std::vector<Eigen::Vector3d>& path, double duration) {
    int n_segments = std::min(10, static_cast<int>(path.size()) - 1);
    
    // 选择关键路径点
    std::vector<Eigen::Vector3d> key_points;
    std::vector<double> segment_durations;
    
    // 均匀选择关键点
    for (int i = 0; i < n_segments + 1; ++i) {
        int idx = static_cast<int>(1.0 * i / n_segments * (path.size() - 1));
        key_points.push_back(path[idx]);
        
        // 根据点间距离调整每段持续时间
        if (i > 0) {
            double dist = (key_points[i] - key_points[i-1]).norm();
            segment_durations.push_back(dist);
        }
    }
    
    // 归一化段持续时间
    double total_dist = 0;
    for (double dist : segment_durations) {
        total_dist += dist;
    }
    
    for (int i = 0; i < segment_durations.size(); ++i) {
        segment_durations[i] = segment_durations[i] / total_dist * duration;
    }
    
    // 存储分段多项式系数
    segment_coeffs_x_.clear();
    segment_coeffs_y_.clear();
    segment_coeffs_z_.clear();
    segment_times_.clear();
    
    // 初始化速度和加速度
    Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();  // 起点速度为0
    Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();  // 起点加速度为0
    Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();    // 终点速度为0
    Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();    // 终点加速度为0
    
    double current_time = 0;
    
    // 计算中间点的速度和加速度估计值（为了保证连续性）
    std::vector<Eigen::Vector3d> velocities(key_points.size(), Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> accelerations(key_points.size(), Eigen::Vector3d::Zero());
    
    // 第一个点和最后一个点的速度和加速度已经设为0
    // 估计中间点的速度（使用有限差分）
    for (int i = 1; i < key_points.size() - 1; ++i) {
        // 使用中心差分估计速度
        Eigen::Vector3d v_prev = (key_points[i] - key_points[i-1]) / segment_durations[i-1];
        Eigen::Vector3d v_next = (key_points[i+1] - key_points[i]) / segment_durations[i];
        velocities[i] = (v_prev + v_next) / 2.0;
        
        // 使用有限差分估计加速度（可选，如果需要更高平滑度）
        if (i > 1 && i < key_points.size() - 2) {
            double dt_prev = segment_durations[i-1];
            double dt_next = segment_durations[i];
            Eigen::Vector3d a_prev = 2.0 * (key_points[i] - key_points[i-1] - v_prev * dt_prev) / (dt_prev * dt_prev);
            Eigen::Vector3d a_next = 2.0 * (key_points[i+1] - key_points[i] - v_next * dt_next) / (dt_next * dt_next);
            accelerations[i] = (a_prev + a_next) / 2.0;
        }
    }
    
    // 为每段计算多项式系数
    for (int i = 0; i < n_segments; ++i) {
        // 获取当前段的边界条件
        Eigen::Vector3d p0 = key_points[i];
        Eigen::Vector3d p1 = key_points[i+1];
        Eigen::Vector3d v0 = velocities[i];
        Eigen::Vector3d v1 = velocities[i+1];
        Eigen::Vector3d a0 = accelerations[i];
        Eigen::Vector3d a1 = accelerations[i+1];
        double T = segment_durations[i];
        
        // 计算x方向的系数
        std::vector<double> x_coeffs = calculateQuinticPolynomialWithConstraints(
            p0.x(), p1.x(), v0.x(), v1.x(), a0.x(), a1.x(), T);
        
        // 计算y方向的系数
        std::vector<double> y_coeffs = calculateQuinticPolynomialWithConstraints(
            p0.y(), p1.y(), v0.y(), v1.y(), a0.y(), a1.y(), T);
        
        // 计算z方向的系数
        std::vector<double> z_coeffs = calculateQuinticPolynomialWithConstraints(
            p0.z(), p1.z(), v0.z(), v1.z(), a0.z(), a1.z(), T);
        
        // 存储系数
        segment_coeffs_x_.push_back(x_coeffs);
        segment_coeffs_y_.push_back(y_coeffs);
        segment_coeffs_z_.push_back(z_coeffs);
        
        current_time += segment_durations[i];
        segment_times_.push_back(current_time);
    }
    ROS_INFO("Path size: %zu, n_segments: %d", path.size(), n_segments);
    ROS_INFO("Total distance: %f", total_dist);
    ROS_INFO("Final segment_times_ size: %zu", segment_times_.size());
    
    // 设置使用分段轨迹标志
    use_segmented_trajectory_ = true;

    // 验证轨迹是否有效
    if (segment_times_.empty()) {
        ROS_WARN("Generated trajectory has no segments");
        use_segmented_trajectory_ = false;
        return false;
    }
    
    return true;
}

// 计算带约束的五次多项式系数（支持任意位置、速度和加速度边界条件）
std::vector<double> CustomTrajectoryPlanner::calculateQuinticPolynomialWithConstraints(
    double p0, double p1, double v0, double v1, double a0, double a1, double T) {
    
    std::vector<double> coeffs(6);
    
    // 直接设置前三个系数（根据起点的位置、速度和加速度）
    coeffs[0] = p0;        // a0 = p0
    coeffs[1] = v0;        // a1 = v0
    coeffs[2] = a0 / 2.0;  // a2 = a0/2
    
    // 计算时间的幂
    double T2 = T * T;    // T^2
    double T3 = T2 * T;   // T^3
    double T4 = T3 * T;   // T^4
    double T5 = T4 * T;   // T^5
    
    // 设置方程组的矩阵和向量
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    
    // 矩阵A的每一行对应一个边界条件
    A << T3, T4, T5,
         3*T2, 4*T3, 5*T4,
         6*T, 12*T2, 20*T3;
    
    // 向量b是右侧的常数项
    b << p1 - p0 - v0*T - a0*T2/2.0,
         v1 - v0 - a0*T,
         a1 - a0;
    
    // 求解线性方程组 A * [a3, a4, a5]^T = b
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    
    // 将解赋值给系数
    coeffs[3] = x(0);  // a3
    coeffs[4] = x(1);  // a4
    coeffs[5] = x(2);  // a5
    
    return coeffs;
}


// /**
//  * 采样轨迹：计算指定时间点的位置和速度
//  * @param t 时间点（秒）
//  * @param pos [输出] 计算得到的位置（3D向量）
//  * @param vel [输出] 计算得到的速度（3D向量）
//  * @return 是否成功采样
//  */
// bool CustomTrajectoryPlanner::sampleTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
//     if (!trajectory_generated_) {
//         std::cerr << "Error: Trajectory not generated yet!" << std::endl;
//         return false;
//     }

//     // 如果时间超出轨迹持续时间，返回终点位置和零速度
//     if (t > duration_) {
//         pos = end_pos_;
//         vel = Eigen::Vector3d::Zero();
//         return true;
//     }

//     // 使用分段轨迹
//     if (use_segmented_trajectory_ && !segment_times_.empty()) {
//         return sampleSegmentedTrajectory(t, pos, vel);
//     }

//     // 使用单段五次多项式轨迹（原有代码）
//     double x = 0.0, y = 0.0, z = 0.0;
//     double vx = 0.0, vy = 0.0, vz = 0.0;

//     for (int i = 0; i < 6; ++i) {
//         double t_power = std::pow(t, i);
//         x += x_coeffs_[i] * t_power;
//         y += y_coeffs_[i] * t_power;
//         z += z_coeffs_[i] * t_power;
//     }

//     vx = x_coeffs_[1] + 2*x_coeffs_[2]*t + 3*x_coeffs_[3]*t*t + 4*x_coeffs_[4]*t*t*t + 5*x_coeffs_[5]*t*t*t*t;
//     vy = y_coeffs_[1] + 2*y_coeffs_[2]*t + 3*y_coeffs_[3]*t*t + 4*y_coeffs_[4]*t*t*t + 5*y_coeffs_[5]*t*t*t*t;
//     vz = z_coeffs_[1] + 2*z_coeffs_[2]*t + 3*z_coeffs_[3]*t*t + 4*z_coeffs_[4]*t*t*t + 5*z_coeffs_[5]*t*t*t*t;

//     pos = Eigen::Vector3d(x, y, z);
//     vel = Eigen::Vector3d(vx, vy, vz);

//     return true;
// }

bool CustomTrajectoryPlanner::sampleSegmentedTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
    // 边界检查
    if (segment_times_.empty()) {
        ROS_ERROR("No segments defined in trajectory!");
        return false;
    }
    
    // 找到当前时间所在的段
    int segment_idx = 0;
    double segment_start_time = 0.0;
    
    // 如果时间超过了最后一段的结束时间，使用最后一段的结束时间
    if (t > segment_times_.back()) {
        ROS_DEBUG("Time %.3f exceeds trajectory duration %.3f", t, segment_times_.back());
        pos = end_pos_;
        vel = Eigen::Vector3d::Zero();
        return true;
    }
    
    // 查找时间所在的段
    for (size_t i = 0; i < segment_times_.size(); ++i) {
        if (t <= segment_times_[i]) {
            segment_idx = i;
            break;
        }
        if (i > 0) {
            segment_start_time = segment_times_[i-1];
        }
    }
    
    // 计算段内相对时间
    double segment_t = t - segment_start_time;
    double segment_duration = (segment_idx > 0) ? 
                             (segment_times_[segment_idx] - segment_times_[segment_idx-1]) : 
                             segment_times_[0];
                             
    // 确保段内时间不超过段持续时间
    if (segment_t > segment_duration) {
        ROS_WARN("Segment time calculation error: t=%.3f, segment_t=%.3f, segment_duration=%.3f",
                t, segment_t, segment_duration);
        segment_t = segment_duration;
    }
    
    // 获取当前段的系数
    if (segment_idx >= segment_coeffs_x_.size()) {
        ROS_ERROR("Segment index %d out of bounds (max: %zu)!", 
                 segment_idx, segment_coeffs_x_.size() - 1);
        return false;
    }
    
    const std::vector<double>& x_coeffs = segment_coeffs_x_[segment_idx];
    const std::vector<double>& y_coeffs = segment_coeffs_y_[segment_idx];
    const std::vector<double>& z_coeffs = segment_coeffs_z_[segment_idx];
    
    // 计算位置和速度
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    
    for (int i = 0; i < 6; ++i) {
        double t_power = std::pow(segment_t, i);
        x += x_coeffs[i] * t_power;
        y += y_coeffs[i] * t_power;
        z += z_coeffs[i] * t_power;
    }
    
    vx = x_coeffs[1] + 2*x_coeffs[2]*segment_t + 3*x_coeffs[3]*segment_t*segment_t + 
         4*x_coeffs[4]*segment_t*segment_t*segment_t + 5*x_coeffs[5]*segment_t*segment_t*segment_t*segment_t;
    vy = y_coeffs[1] + 2*y_coeffs[2]*segment_t + 3*y_coeffs[3]*segment_t*segment_t + 
         4*y_coeffs[4]*segment_t*segment_t*segment_t + 5*y_coeffs[5]*segment_t*segment_t*segment_t*segment_t;
    vz = z_coeffs[1] + 2*z_coeffs[2]*segment_t + 3*z_coeffs[3]*segment_t*segment_t + 
         4*z_coeffs[4]*segment_t*segment_t*segment_t + 5*z_coeffs[5]*segment_t*segment_t*segment_t*segment_t;
    
    pos = Eigen::Vector3d(x, y, z);
    vel = Eigen::Vector3d(vx, vy, vz);
    
    return true;
}


/**
 * 计算五次多项式系数：解析求解边界条件方程组
 * @param start 起始位置
 * @param end 终点位置
 * @param duration 持续时间
 * @param coefficients [输出] 计算得到的多项式系数数组
 * 
 * 五次多项式形式: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 * 
 * 边界条件:
 * 1. p(0) = start            - 起点位置
 * 2. p(T) = end              - 终点位置
 * 3. p'(0) = 0               - 起点速度为零
 * 4. p'(T) = 0               - 终点速度为零
 * 5. p''(0) = 0              - 起点加速度为零
 * 6. p''(T) = 0              - 终点加速度为零
 */
void CustomTrajectoryPlanner::calculateQuinticPolynomial(double start, double end, double duration,
                                                      std::vector<double>& coefficients) {
    // 计算时间的幂
    double T = duration;
    double T2 = T * T;    // T^2
    double T3 = T2 * T;   // T^3
    double T4 = T3 * T;   // T^4
    double T5 = T4 * T;   // T^5

    // 根据边界条件直接计算系数
    // 从边界条件可以直接得到前三个系数
    coefficients[0] = start;  // a0 = start (位置起点)
    coefficients[1] = 0.0;    // a1 = 0 (起点速度为0)
    coefficients[2] = 0.0;    // a2 = 0 (起点加速度为0)

    // 计算剩余系数 a3, a4, a5
    // 这些系数是通过求解线性方程组得到的解析解
    // 方程组来自剩余的边界条件:
    // 1. a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5*T^5 = end
    // 2. a1 + 2*a2*T + 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4 = 0
    // 3. 2*a2 + 6*a3*T + 12*a4*T^2 + 20*a5*T^3 = 0
    coefficients[3] = 10.0 * (end - start) / T3;    // a3
    coefficients[4] = -15.0 * (end - start) / T4;   // a4
    coefficients[5] = 6.0 * (end - start) / T5;     // a5
}

/**
 * 获取生成的路径点
 * @return 路径点向量
 */
const std::vector<Eigen::Vector3d>& CustomTrajectoryPlanner::getPath() const {
    return path_;
}

/////////////////////////////////////////////////////////////// custom_trajectory_planner.cpp (新增部分)

// 构造函数增加初始化
CustomTrajectoryPlanner::CustomTrajectoryPlanner()
    : duration_(0.0), trajectory_generated_(false),
      attractive_gain_(1.0), repulsive_gain_(100.0), 
      safe_distance_(1.0), goal_threshold_(0.2),
      use_segmented_trajectory_(false),
      astar_planner_(new AStarPlanner()),
      minisnap_trajectory_(new MinisnapTrajectory()) {
    // 初始化多项式系数向量
    x_coeffs_.resize(6, 0.0);
    y_coeffs_.resize(6, 0.0);
    z_coeffs_.resize(6, 0.0);
}

// 析构函数释放资源
CustomTrajectoryPlanner::~CustomTrajectoryPlanner() {
    if (astar_planner_) {
        delete astar_planner_;
        astar_planner_ = nullptr;
    }
    
    if (minisnap_trajectory_) {
        delete minisnap_trajectory_;
        minisnap_trajectory_ = nullptr;
    }
}

// 使用A*和Minisnap生成避障轨迹
bool CustomTrajectoryPlanner::generatePathWithAStarAndMinisnap(
    // 三维
    // const Eigen::Vector3d& start_pos,
    // const Eigen::Vector3d& end_pos,
    // double duration,
    // double grid_resolution) {
    
    // // 保存起点和终点
    // start_pos_ = start_pos;
    // end_pos_ = end_pos;
    // duration_ = duration;
    
    // // 设置A*规划器参数
    // astar_planner_->setObstacles(obstacles_);
    
    // // 设置规划空间范围（根据起点和终点动态调整）
    // Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();
    // Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();
    
    // // 计算包含起点和终点的边界，并添加一定的边界
    // min_bounds.x() = std::min(start_pos.x(), end_pos.x()) - 5.0;
    // min_bounds.y() = std::min(start_pos.y(), end_pos.y()) - 5.0;
    // min_bounds.z() = std::min(start_pos.z(), end_pos.z()) - 2.0;
    
    // max_bounds.x() = std::max(start_pos.x(), end_pos.x()) + 5.0;
    // max_bounds.y() = std::max(start_pos.y(), end_pos.y()) + 5.0;
    // max_bounds.z() = std::max(start_pos.z(), end_pos.z()) + 2.0;
    
    // astar_planner_->setPlanningBounds(min_bounds, max_bounds);
    
    // // 使用A*规划路径
    // std::vector<Eigen::Vector3d> astar_path;
    // bool astar_success = astar_planner_->plan(start_pos, end_pos, astar_path);
    
    // if (!astar_success) {
    //     std::cerr << "A* planning failed! Falling back to direct trajectory." << std::endl;
    //     return generateTrajectory(start_pos, end_pos, duration);
    // }
    // else
    // {
    //     std::cerr << "A* planning success." << std::endl;
    // }
    
    // // 保存A*生成的路径
    // path_ = astar_path;
    
    // // 使用Minisnap平滑路径
    // bool minisnap_success = minisnap_trajectory_->generateTrajectory(astar_path, duration);
    
    // if (!minisnap_success) {
    //     std::cerr << "Minisnap trajectory generation failed! Falling back to direct trajectory." << std::endl;
    //     return generateTrajectory(start_pos, end_pos, duration);
    // }
    
    // trajectory_generated_ = true;
    // return true;

    // 二维
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    double duration,
    double grid_resolution) {
    
    // 保存起点和终点
    start_pos_ = start_pos;
    end_pos_ = end_pos;
    duration_ = duration;
    
    // 保存固定的Z高度
    double fixed_z = start_pos.z();
    
    // 设置A*规划器参数
    astar_planner_->setObstacles(obstacles_);
    
    // 设置规划空间范围（根据起点和终点动态调整）- 只考虑XY平面
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();
    
    // 计算包含起点和终点的边界，并添加一定的边界
    min_bounds.x() = std::min(start_pos.x(), end_pos.x()) - 5.0;
    min_bounds.y() = std::min(start_pos.y(), end_pos.y()) - 5.0;
    min_bounds.z() = fixed_z - 0.1;  // Z方向的边界非常窄，因为我们只在XY平面规划
    
    max_bounds.x() = std::max(start_pos.x(), end_pos.x()) + 5.0;
    max_bounds.y() = std::max(start_pos.y(), end_pos.y()) + 5.0;
    max_bounds.z() = fixed_z + 0.1;  // Z方向的边界非常窄，因为我们只在XY平面规划
    
    astar_planner_->setPlanningBounds(min_bounds, max_bounds);
    
    // 使用A*规划路径 - 修改后的A*只在XY平面规划
    std::vector<Eigen::Vector3d> astar_path;
    bool astar_success = astar_planner_->plan(start_pos, end_pos, astar_path);
    
    if (!astar_success) {
        std::cerr << "A* planning failed! Falling back to direct trajectory." << std::endl;
        return generateTrajectory(start_pos, end_pos, duration);
    }
    
    // 确保所有路径点的Z高度一致
    for (auto& point : astar_path) {
        point.z() = fixed_z;
    }
    
    // 保存A*生成的路径
    path_ = astar_path;
    
    // 使用Minisnap平滑路径
    bool minisnap_success = minisnap_trajectory_->generateTrajectory(astar_path, duration);
    
    if (!minisnap_success) {
        std::cerr << "Minisnap trajectory generation failed! Falling back to direct trajectory." << std::endl;
        return generateTrajectory(start_pos, end_pos, duration);
    }
    
    trajectory_generated_ = true;
    return true;
}

void CustomTrajectoryPlanner::startPublishingPathPoints(ros::NodeHandle& nh, const std::string& uav_name) {
    // 如果路径为空，则不发布
    if (path_.empty()) {
        std::cerr << "No path to publish!" << std::endl;
        return;
    }
    
    // 如果该UAV的发布器不存在，则创建一个
    if (path_point_pubs_.find(uav_name) == path_point_pubs_.end()) {
        std::string topic_name = "/" + uav_name + "/planned_path_points";
        path_point_pubs_[uav_name] = nh.advertise<geometry_msgs::PointStamped>(topic_name, 10, true);
    }
    
    // 如果已经有定时器在运行，先停止它
    if (path_publish_timers_.find(uav_name) != path_publish_timers_.end()) {
        path_publish_timers_[uav_name].stop();
    }
    
    // 创建一个定时器，定期发布路径点
    // 捕获this和uav_name，使回调函数能访问类成员和UAV名称
    path_publish_timers_[uav_name] = nh.createTimer(
        ros::Duration(1.0),  // 每1秒发布一次所有点
        [this, uav_name](const ros::TimerEvent&) {
            // 发布所有路径点
            for (size_t i = 0; i < this->path_.size(); ++i) {
                geometry_msgs::PointStamped point_msg;
                point_msg.header.frame_id = "map";
                point_msg.header.stamp = ros::Time::now();
                
                point_msg.point.x = this->path_[i].x();
                point_msg.point.y = this->path_[i].y();
                point_msg.point.z = this->path_[i].z();
                
                this->path_point_pubs_[uav_name].publish(point_msg);
                
                // 添加一个短暂的延迟，确保消息能被接收
                ros::Duration(0.01).sleep();
            }
        }
    );
}


// 修改采样轨迹方法，支持Minisnap
bool CustomTrajectoryPlanner::sampleTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
    if (!trajectory_generated_) {
        return false;
    }
    
    // 检查时间范围
    double total_time = 0;
    for (const auto& segment_time : segment_times_) {
        total_time += segment_time;
    }
    // 优先使用 Minisnap 轨迹的总时间（如果可用）
    if (minisnap_trajectory_ && minisnap_trajectory_->isTrajectoryGenerated()) {
        total_time = minisnap_trajectory_->getTotalDuration();
    } 
    else if (use_segmented_trajectory_ && !segment_times_.empty()) {
        total_time = segment_times_.back();  // 最后一个时间点就是总时间
    } 


    // if (t < 0 || t > total_time) {
    //     ROS_ERROR("Time %.2f is outside trajectory range [0, %.2f]", t, total_time);
    //     return false;
    // }
    
    // 处理时间超出范围的情况
    if (t < 0) {
        ROS_WARN("Time %.3f is negative, using t=0", t);
        t = 0;
    }
    
    if (t > duration_) {
        ROS_DEBUG("Time %.3f exceeds trajectory duration %.3f, returning end point", t, duration_);
        pos = end_pos_;
        vel = Eigen::Vector3d::Zero();
        return true;
    }
    
    // 如果使用Minisnap轨迹
    if (minisnap_trajectory_) {
        bool minisnap_success = minisnap_trajectory_->sampleTrajectory(t, pos, vel);
        if (minisnap_success) {
            return true;
        } else {
            ROS_WARN("Minisnap sampling failed at t=%.3f, falling back to polynomial trajectory", t);
        }
    }
    
    // 使用分段轨迹
    if (use_segmented_trajectory_ && !segment_times_.empty()) {
        return sampleSegmentedTrajectory(t, pos, vel);
    }
    
    // 使用单段五次多项式轨迹
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    
    for (int i = 0; i < 6; ++i) {
        double t_power = std::pow(t, i);
        x += x_coeffs_[i] * t_power;
        y += y_coeffs_[i] * t_power;
        z += z_coeffs_[i] * t_power;
    }
    
    vx = x_coeffs_[1] + 2*x_coeffs_[2]*t + 3*x_coeffs_[3]*t*t + 4*x_coeffs_[4]*t*t*t + 5*x_coeffs_[5]*t*t*t*t;
    vy = y_coeffs_[1] + 2*y_coeffs_[2]*t + 3*y_coeffs_[3]*t*t + 4*y_coeffs_[4]*t*t*t + 5*y_coeffs_[5]*t*t*t*t;
    vz = z_coeffs_[1] + 2*z_coeffs_[2]*t + 3*z_coeffs_[3]*t*t + 4*z_coeffs_[4]*t*t*t + 5*z_coeffs_[5]*t*t*t*t;
    
    pos = Eigen::Vector3d(x, y, z);
    vel = Eigen::Vector3d(vx, vy, vz);
    
    return true;
}


// 测试
// 设置编队偏移量
void CustomTrajectoryPlanner::setFormationOffsets(const std::map<std::string, Eigen::Vector3d>& offsets) {
    formation_offsets_ = offsets;
    if (astar_planner_) {
        astar_planner_->setFormationOffsets(offsets);
    }
}

// 使用考虑编队约束的A*和Minisnap生成避障轨迹
bool CustomTrajectoryPlanner::generatePathWithFormationConstraints(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    double duration,
    double grid_resolution) {
    
    // 保存起点和终点
    start_pos_ = start_pos;
    end_pos_ = end_pos;
    duration_ = duration;
    
    // 保存固定的Z高度
    double fixed_z = start_pos.z();
    
    // 设置A*规划器参数
    astar_planner_->setObstacles(obstacles_);
    
    // 设置规划空间范围（根据起点和终点动态调整）- 只考虑XY平面
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();
    
    // 计算包含起点和终点的边界，并添加一定的边界
    min_bounds.x() = std::min(start_pos.x(), end_pos.x()) - 5.0;
    min_bounds.y() = std::min(start_pos.y(), end_pos.y()) - 5.0;
    min_bounds.z() = fixed_z - 0.1;  // Z方向的边界非常窄，因为我们只在XY平面规划
    
    max_bounds.x() = std::max(start_pos.x(), end_pos.x()) + 5.0;
    max_bounds.y() = std::max(start_pos.y(), end_pos.y()) + 5.0;
    max_bounds.z() = fixed_z + 0.1;  // Z方向的边界非常窄，因为我们只在XY平面规划
    
    astar_planner_->setPlanningBounds(min_bounds, max_bounds);
    
    // 使用考虑编队约束的A*规划路径
    std::vector<Eigen::Vector3d> astar_path;
    bool astar_success = astar_planner_->planWithFormationConstraints(start_pos, end_pos, astar_path);
    
    if (!astar_success) {
        std::cerr << "Formation-aware A* planning failed! Falling back to regular A*." << std::endl;
        return generatePathWithAStarAndMinisnap(start_pos, end_pos, duration, grid_resolution);
    }
    
    // 确保所有路径点的Z高度一致
    for (auto& point : astar_path) {
        point.z() = fixed_z;
    }
    
    // 保存A*生成的路径
    path_ = astar_path;
    
    // 使用Minisnap平滑路径
    bool minisnap_success = minisnap_trajectory_->generateTrajectory(astar_path, duration);
    
    if (!minisnap_success) {
        std::cerr << "Minisnap trajectory generation failed! Falling back to direct trajectory." << std::endl;
        return generateTrajectory(start_pos, end_pos, duration);
    }
    
    trajectory_generated_ = true;
    return true;
}

// 预规划
// 在custom_trajectory_planner.cpp中实现
bool CustomTrajectoryPlanner::generateFollowerTrajectory(
    const std::vector<Eigen::Vector3d>& leader_path,
    const Eigen::Vector3d& offset,
    double duration) {
    
    // 1. 获取领导者的路径点
    if (leader_path.empty()) {
        ROS_ERROR("Leader path is empty!");
        return false;
    }
    
    // 2. 为跟随者生成初始路径点(添加偏移量)
    std::vector<Eigen::Vector3d> follower_path;
    for (const auto& point : leader_path) {
        follower_path.push_back(point + offset);
    }
    
    // // 3. 检查碰撞
    // std::vector<int> collision_indices;
    // for (size_t i = 0; i < follower_path.size(); ++i) {
    //     if (!astar_planner_->isCollisionFree(follower_path[i])) {
    //         collision_indices.push_back(i);
    //     }
    // }
    
    // // 4. 如果有碰撞，进行局部重规划
    // if (!collision_indices.empty()) {
    //     ROS_WARN("Obstacle Planning");
    //     // 找到第一个碰撞点和最后一个碰撞点
    //     int first_collision = collision_indices.front();
    //     int last_collision = collision_indices.back();
        
    //     // 确定局部规划的起点和终点
    //     // 往前找一个安全点作为起点
    //     int start_idx = std::max(0, first_collision - 5);
    //     // 往后找一个安全点作为终点
    //     int end_idx = std::min((int)follower_path.size() - 1, last_collision + 5);
        
    //     // 确保起点和终点是安全的
    //     while (start_idx < first_collision && !astar_planner_->isCollisionFree(follower_path[start_idx])) {
    //         start_idx++;
    //     }
        
    //     while (end_idx > last_collision && !astar_planner_->isCollisionFree(follower_path[end_idx])) {
    //         end_idx--;
    //     }
        
    //     // 如果找不到安全的起点或终点，返回失败
    //     if (start_idx >= first_collision || end_idx <= last_collision) {
    //         ROS_ERROR("Cannot find safe start/end points for local replanning!");
    //         return false;
    //     }
        
    //     // 局部A*规划
    //     std::vector<Eigen::Vector3d> local_path;
    //     bool success = astar_planner_->plan(
    //         follower_path[start_idx], 
    //         follower_path[end_idx], 
    //         local_path);
            
    //     if (!success) {
    //         ROS_ERROR("Local replanning failed!");
    //         return false;
    //     }
        
    //     // 替换原路径中的碰撞部分
    //     std::vector<Eigen::Vector3d> new_path;
    //     for (int i = 0; i <= start_idx; ++i) {
    //         new_path.push_back(follower_path[i]);
    //     }
        
    //     // 添加局部规划路径(跳过第一个点，因为已经添加了)
    //     for (size_t i = 1; i < local_path.size(); ++i) {
    //         new_path.push_back(local_path[i]);
    //     }
        
    //     // 添加剩余路径点
    //     for (size_t i = end_idx + 1; i < follower_path.size(); ++i) {
    //         new_path.push_back(follower_path[i]);
    //     }
        
    //     follower_path = new_path;
    // }
    
    // 5. 使用Minisnap平滑路径
    path_ = follower_path;
    bool minisnap_success = minisnap_trajectory_->generateTrajectory(follower_path, duration);
    
    if (!minisnap_success) {
        ROS_ERROR("Failed to generate Minisnap trajectory for follower!");
        return false;
    }
    
    start_pos_ = follower_path.front();
    end_pos_ = follower_path.back();
    duration_ = duration;
    trajectory_generated_ = true;
    
    return true;
}

void CustomTrajectoryPlanner::publishPathAsRosMsg(const ros::Publisher& pub) const {
    // 如果路径为空，则不发布
    if (path_.empty()) {
        ROS_WARN("Cannot publish path: Path is empty");
        return;
    }
    
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";  // 使用适合您系统的坐标系
    
    // 将路径点转换为PoseStamped消息
    for (const auto& point : path_) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        
        // 设置默认方向（朝向z轴）
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose);
    }
    
    // 发布路径消息
    pub.publish(path_msg);
    ROS_INFO("Published path with %zu points", path_msg.poses.size());
}

// 将ROS路径消息转换为Eigen向量数组
std::vector<Eigen::Vector3d> CustomTrajectoryPlanner::rosPathMsgToEigen(const nav_msgs::Path& path_msg) {
    std::vector<Eigen::Vector3d> eigen_path;
    eigen_path.reserve(path_msg.poses.size());
    
    for (const auto& pose : path_msg.poses) {
        Eigen::Vector3d point(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        );
        eigen_path.push_back(point);
    }
    
    return eigen_path;
}

// 在CustomTrajectoryPlanner中添加更强健的检查
bool CustomTrajectoryPlanner::isTrajectoryGenerated() const {
    if (!trajectory_generated_) {
        return false;
    }
    
    // 额外检查轨迹对象是否有效
    if (!minisnap_trajectory_) {
        ROS_ERROR("Trajectory marked as generated but minisnap_trajectory_ is null!");
        return false;
    }

    return true;
}

bool CustomTrajectoryPlanner::generatePathWithInitialVelocity(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    const Eigen::Vector3d& start_vel,
    double duration) {
    
    // 保存起点、终点和持续时间
    start_pos_ = start_pos;
    end_pos_ = end_pos;
    duration_ = duration;
    
    // 保存固定的Z高度
    double fixed_z = start_pos.z();
    
    // 设置A*规划器参数
    astar_planner_->setObstacles(obstacles_);
    
    // 设置规划空间范围（根据起点和终点动态调整）- 只考虑XY平面
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();
    
    // 计算包含起点和终点的边界，并添加一定的边界
    min_bounds.x() = std::min(start_pos.x(), end_pos.x()) - 5.0;
    min_bounds.y() = std::min(start_pos.y(), end_pos.y()) - 5.0;
    min_bounds.z() = fixed_z - 0.1;  // Z方向的边界非常窄，因为我们只在XY平面规划
    
    max_bounds.x() = std::max(start_pos.x(), end_pos.x()) + 5.0;
    max_bounds.y() = std::max(start_pos.y(), end_pos.y()) + 5.0;
    max_bounds.z() = fixed_z + 0.1;  // Z方向的边界非常窄，因为我们只在XY平面规划
    
    astar_planner_->setPlanningBounds(min_bounds, max_bounds);
    
    // 使用A*规划路径 - 修改后的A*只在XY平面规划
    std::vector<Eigen::Vector3d> astar_path;
    bool astar_success = astar_planner_->plan(start_pos, end_pos, astar_path);
    
    if (!astar_success) {
        ROS_ERROR("A* planning failed! Falling back to direct trajectory.");
        return generateTrajectory(start_pos, end_pos, duration);
    }
    
    // 确保所有路径点的Z高度一致
    for (auto& point : astar_path) {
        point.z() = fixed_z;
    }
    
    // 保存A*生成的路径
    path_ = astar_path;
    
    // 使用Minisnap平滑路径，并考虑初始速度
    bool minisnap_success = minisnap_trajectory_->generateTrajectoryWithInitialVelocity(
        astar_path, start_vel, Eigen::Vector3d::Zero(), duration);
    
    if (!minisnap_success) {
        ROS_ERROR("Minisnap trajectory generation failed! Falling back to direct trajectory.");
        return generateTrajectory(start_pos, end_pos, duration);
    }
    
    trajectory_generated_ = true;
    return true;
}

