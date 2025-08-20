// minisnap_trajectory.cpp
#include "offboard_pkg/minisnap_trajectory.h"
#include <iostream>
#include <cmath>

MinisnapTrajectory::MinisnapTrajectory() : total_duration_(0.0), trajectory_generated_(false) {}

MinisnapTrajectory::~MinisnapTrajectory() {}

// 在MinisnapTrajectory::generateTrajectory函数中，确保在返回前设置trajectory_generated_标志
bool MinisnapTrajectory::generateTrajectory(const std::vector<Eigen::Vector3d>& waypoints, 
    double duration, 
    int derivative_order) {
    if (waypoints.size() < 2) {
        ROS_ERROR("At least 2 waypoints are required for trajectory generation!");
        trajectory_generated_ = false;
        return false;
    }

    // 重要：先将trajectory_generated_设置为false
    trajectory_generated_ = false;
    
    waypoints_ = waypoints;
    // 打印路径点进行调试
    ROS_INFO("Minisnap waypoints (%zu points):", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        ROS_INFO("Waypoint %zu: (%.2f, %.2f, %.2f)", i, waypoints_[i].x(), waypoints_[i].y(), waypoints_[i].z());
    }

    total_duration_ = duration;

    // 计算多项式系数
    try {
        computeCoefficients(waypoints, duration, derivative_order);
        
        // 检查系数是否有效
        if (x_coeffs_.empty() || y_coeffs_.empty() || z_coeffs_.empty()) {
            ROS_ERROR("Failed to compute trajectory coefficients!");
            return false;
        }
        
        // 现在设置为true，表示轨迹已生成
        trajectory_generated_ = true;
        ROS_WARN("trajectory_generated_ is true");
        // 验证生成的轨迹
        validateTrajectory();
        
        return true;
    } 
    catch (const std::exception& e) {
        ROS_ERROR("Exception during trajectory generation: %s", e.what());
        trajectory_generated_ = false;
        return false;
    }
}

bool MinisnapTrajectory::generateTrajectoryWithInitialVelocity(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& end_vel,
    double duration,
    int derivative_order) {
    
    if (waypoints.size() < 2) {
        ROS_ERROR("At least 2 waypoints are required for trajectory generation!");
        trajectory_generated_ = false;
        return false;
    }

    // 重要：先将trajectory_generated_设置为false
    trajectory_generated_ = false;
    
    waypoints_ = waypoints;
    start_vel_ = start_vel;  // 保存初始速度
    end_vel_ = end_vel;      // 保存终点速度
    
    // 打印路径点和速度约束进行调试
    ROS_INFO("Minisnap waypoints (%zu points) with velocity constraints:", waypoints_.size());
    ROS_INFO("Start velocity: (%.2f, %.2f, %.2f)", start_vel.x(), start_vel.y(), start_vel.z());
    ROS_INFO("End velocity: (%.2f, %.2f, %.2f)", end_vel.x(), end_vel.y(), end_vel.z());
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        ROS_INFO("Waypoint %zu: (%.2f, %.2f, %.2f)", i, waypoints_[i].x(), waypoints_[i].y(), waypoints_[i].z());
    }

    total_duration_ = duration;

    // 计算多项式系数，考虑速度约束
    try {
        computeCoefficientsWithVelocity(waypoints, start_vel, end_vel, duration, derivative_order);
        
        // 检查系数是否有效
        if (x_coeffs_.empty() || y_coeffs_.empty() || z_coeffs_.empty()) {
            ROS_ERROR("Failed to compute trajectory coefficients!");
            return false;
        }
        
        // 现在设置为true，表示轨迹已生成
        trajectory_generated_ = true;
        ROS_INFO("Trajectory with initial velocity generated successfully");
        
        // 验证生成的轨迹
        validateTrajectory();
        
        return true;
    } 
    catch (const std::exception& e) {
        ROS_ERROR("Exception during trajectory generation: %s", e.what());
        trajectory_generated_ = false;
        return false;
    }
}

void MinisnapTrajectory::computeCoefficientsWithVelocity(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& end_vel,
    double duration,
    int derivative_order) {
    
    int n_segments = waypoints.size() - 1;
    int n_coeffs = 2 * derivative_order;  // 每段多项式的系数数量
    
    // 分配分段时间
    segment_times_ = allocateTime(waypoints, duration);
    
    // 打印分段时间信息
    ROS_INFO("Segment times:");
    double total_time = 0.0;
    for (size_t i = 0; i < segment_times_.size(); ++i) {
        total_time += segment_times_[i];
        ROS_INFO("  Segment %zu: %.2f s (%.2f to %.2f)", 
                 i, segment_times_[i], total_time - segment_times_[i], total_time);
    }
    
    // 确保总时间与预期一致
    if (std::abs(total_time - duration) > 1e-3) {
        ROS_WARN("Total segment time (%.2f) differs from requested duration (%.2f)", 
                 total_time, duration);
        // 调整总持续时间以匹配实际分段时间之和
        total_duration_ = total_time;
    }
    
    // 为每个维度创建QP问题
    for (int dim = 0; dim < 3; ++dim) {
        // 设置QP问题的大小
        int n_vars = n_segments * n_coeffs;
        
        // 创建H矩阵（目标函数的二次项）
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
        
        // 创建g向量（目标函数的一次项）
        Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars);
        
        // 创建约束矩阵和向量
        int n_constraints = 2 * derivative_order + (n_segments - 1) * (derivative_order * 2 + 1);
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_vars);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(n_constraints);
        
        // 填充H矩阵（最小化snap）
        for (int i = 0; i < n_segments; ++i) {
            double T = segment_times_[i];
            
            // 对于每段多项式，计算snap的积分
            for (int j = 4; j < n_coeffs; ++j) {
                for (int k = 4; k < n_coeffs; ++k) {
                    double coeff = 0.0;
                    int p1 = j - 4;
                    int p2 = k - 4;
                    coeff = (p1 + 1) * (p1 + 2) * (p1 + 3) * (p1 + 4) * 
                            (p2 + 1) * (p2 + 2) * (p2 + 3) * (p2 + 4) * 
                            std::pow(T, p1 + p2 + 1) / (p1 + p2 + 1);
                    
                    H(i * n_coeffs + j, i * n_coeffs + k) = coeff;
                }
            }
        }
        
        // 填充约束矩阵
        int constraint_idx = 0;
        
        // 起点约束
        for (int d = 0; d < derivative_order; ++d) {
            // 位置约束
            if (d == 0) {
                A(constraint_idx, 0) = 1.0;  // p(0) = waypoint[0]
                b(constraint_idx) = waypoints[0](dim);
            } 
            // 速度约束 - 使用提供的初始速度
            else if (d == 1) {
                A(constraint_idx, 1) = 1.0;  // p'(0) = start_vel
                b(constraint_idx) = start_vel(dim);
            }
            // 高阶导数约束为0
            else {
                for (int j = d; j < n_coeffs; ++j) {
                    double coeff = 1.0;
                    for (int k = 0; k < d; ++k) {
                        coeff *= (j - k);
                    }
                    A(constraint_idx, j) = coeff * std::pow(0.0, j - d);
                }
                b(constraint_idx) = 0.0;  // 高阶导数为0
            }
            constraint_idx++;
        }
        
        // 终点约束
        for (int d = 0; d < derivative_order; ++d) {
            int last_segment = n_segments - 1;
            double T = segment_times_[last_segment];
            
            // 位置约束
            if (d == 0) {
                for (int j = 0; j < n_coeffs; ++j) {
                    A(constraint_idx, last_segment * n_coeffs + j) = std::pow(T, j);
                }
                b(constraint_idx) = waypoints.back()(dim);
            } 
            // 速度约束 - 使用提供的终点速度
            else if (d == 1) {
                for (int j = 1; j < n_coeffs; ++j) {
                    A(constraint_idx, last_segment * n_coeffs + j) = j * std::pow(T, j-1);
                }
                b(constraint_idx) = end_vel(dim);
            }
            // 高阶导数约束为0
            else {
                for (int j = d; j < n_coeffs; ++j) {
                    double coeff = 1.0;
                    for (int k = 0; k < d; ++k) {
                        coeff *= (j - k);
                    }
                    A(constraint_idx, last_segment * n_coeffs + j) = coeff * std::pow(T, j - d);
                }
                b(constraint_idx) = 0.0;  // 高阶导数为0
            }
            constraint_idx++;
        }
        
        // 中间点约束
        for (int i = 0; i < n_segments - 1; ++i) {
            double T = segment_times_[i];
            
            // 连续性约束
            for (int d = 0; d < derivative_order * 2; ++d) {
                if (d == 0) {
                    // 位置约束
                    for (int j = 0; j < n_coeffs; ++j) {
                        A(constraint_idx, i * n_coeffs + j) = std::pow(T, j);
                    }
                    A(constraint_idx, (i + 1) * n_coeffs) = -1.0;  // p_i(T) = p_{i+1}(0)
                    b(constraint_idx) = 0.0;
                } else {
                    // 高阶导数连续性
                    for (int j = d; j < n_coeffs; ++j) {
                        double coeff = 1.0;
                        for (int k = 0; k < d; ++k) {
                            coeff *= (j - k);
                        }
                        A(constraint_idx, i * n_coeffs + j) = coeff * std::pow(T, j - d);
                    }
                    
                    for (int j = d; j < n_coeffs; ++j) {
                        double coeff = 1.0;
                        for (int k = 0; k < d; ++k) {
                            coeff *= (j - k);
                        }
                        A(constraint_idx, (i + 1) * n_coeffs + j) = -coeff * std::pow(0.0, j - d);
                    }
                    b(constraint_idx) = 0.0;
                }
                constraint_idx++;
            }
            
            // 中间点位置约束
            for (int j = 0; j < n_coeffs; ++j) {
                A(constraint_idx, i * n_coeffs + j) = std::pow(T, j);
            }
            b(constraint_idx) = waypoints[i + 1](dim);
            constraint_idx++;
        }
        
        // 求解QP问题
        Eigen::MatrixXd A_t = A.transpose();
        Eigen::MatrixXd H_inv;
        
        // 增强数值稳定性
        Eigen::MatrixXd H_reg = H + Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-8;
        
        if (H_reg.determinant() > 1e-8) {
            H_inv = H_reg.inverse();
        } else {
            // 使用SVD伪逆
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_reg, Eigen::ComputeThinU | Eigen::ComputeThinV);
            double epsilon = 1e-10;  // 奇异值阈值
            Eigen::VectorXd S = svd.singularValues();
            Eigen::VectorXd S_inv = S;
            for (int i = 0; i < S.size(); ++i) {
                if (S(i) > epsilon) {
                    S_inv(i) = 1.0 / S(i);
                } else {
                    S_inv(i) = 0.0;
                }
            }
            H_inv = svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
        }
        
        Eigen::MatrixXd temp = A * H_inv * A_t;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(temp, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd lambda = svd.solve(A * H_inv * g - b);
        Eigen::VectorXd x = H_inv * (g - A_t * lambda);
        
        // 提取系数
        std::vector<std::vector<double>> coeffs(n_segments, std::vector<double>(n_coeffs));
        for (int i = 0; i < n_segments; ++i) {
            for (int j = 0; j < n_coeffs; ++j) {
                coeffs[i][j] = x(i * n_coeffs + j);
            }
        }
        
        // 存储系数
        if (dim == 0) x_coeffs_ = coeffs;
        else if (dim == 1) y_coeffs_ = coeffs;
        else z_coeffs_ = coeffs;
    }
    
    // 检查系数是否合理
    checkCoefficientsWithVelocity();
}

void MinisnapTrajectory::checkCoefficientsWithVelocity() {
    if (x_coeffs_.empty() || y_coeffs_.empty() || z_coeffs_.empty()) {
        ROS_WARN("Empty coefficient vectors!");
        return;
    }
    
    // 检查第一段多项式在t=0时的位置和速度
    double x0 = evaluatePolynomial(x_coeffs_[0], 0.0, 0);
    double y0 = evaluatePolynomial(y_coeffs_[0], 0.0, 0);
    double z0 = evaluatePolynomial(z_coeffs_[0], 0.0, 0);
    
    double vx0 = evaluatePolynomial(x_coeffs_[0], 0.0, 1);
    double vy0 = evaluatePolynomial(y_coeffs_[0], 0.0, 1);
    double vz0 = evaluatePolynomial(z_coeffs_[0], 0.0, 1);
    
    // 检查位置误差
    double x_error = std::abs(x0 - waypoints_[0].x());
    double y_error = std::abs(y0 - waypoints_[0].y());
    double z_error = std::abs(z0 - waypoints_[0].z());
    
    // 检查速度误差
    double vx_error = std::abs(vx0 - start_vel_.x());
    double vy_error = std::abs(vy0 - start_vel_.y());
    double vz_error = std::abs(vz0 - start_vel_.z());
    
    if (x_error > 1e-3 || y_error > 1e-3 || z_error > 1e-3) {
        ROS_WARN("Start point mismatch: computed=(%.4f, %.4f, %.4f), waypoint=(%.4f, %.4f, %.4f)",
                 x0, y0, z0, waypoints_[0].x(), waypoints_[0].y(), waypoints_[0].z());
    } else {
        ROS_INFO("Start point matches waypoint (error < 0.001)");
    }
    
    if (vx_error > 1e-3 || vy_error > 1e-3 || vz_error > 1e-3) {
        ROS_WARN("Start velocity mismatch: computed=(%.4f, %.4f, %.4f), target=(%.4f, %.4f, %.4f)",
                 vx0, vy0, vz0, start_vel_.x(), start_vel_.y(), start_vel_.z());
    } else {
        ROS_INFO("Start velocity matches target (error < 0.001)");
    }
    
    // 检查最后一段多项式在t=T时的位置和速度
    int last_segment = x_coeffs_.size() - 1;
    double T = segment_times_[last_segment];
    
    double xf = evaluatePolynomial(x_coeffs_[last_segment], T, 0);
    double yf = evaluatePolynomial(y_coeffs_[last_segment], T, 0);
    double zf = evaluatePolynomial(z_coeffs_[last_segment], T, 0);
    
    double vxf = evaluatePolynomial(x_coeffs_[last_segment], T, 1);
    double vyf = evaluatePolynomial(y_coeffs_[last_segment], T, 1);
    double vzf = evaluatePolynomial(z_coeffs_[last_segment], T, 1);
    
    // 检查位置误差
    x_error = std::abs(xf - waypoints_.back().x());
    y_error = std::abs(yf - waypoints_.back().y());
    z_error = std::abs(zf - waypoints_.back().z());
    
    // 检查速度误差
    vx_error = std::abs(vxf - end_vel_.x());
    vy_error = std::abs(vyf - end_vel_.y());
    vz_error = std::abs(vzf - end_vel_.z());
    
    if (x_error > 1e-3 || y_error > 1e-3 || z_error > 1e-3) {
        ROS_WARN("End point mismatch: computed=(%.4f, %.4f, %.4f), waypoint=(%.4f, %.4f, %.4f)",
                 xf, yf, zf, waypoints_.back().x(), waypoints_.back().y(), waypoints_.back().z());
    } else {
        ROS_INFO("End point matches waypoint (error < 0.001)");
    }
    
    if (vx_error > 1e-3 || vy_error > 1e-3 || vz_error > 1e-3) {
        ROS_WARN("End velocity mismatch: computed=(%.4f, %.4f, %.4f), target=(%.4f, %.4f, %.4f)",
                 vxf, vyf, vzf, end_vel_.x(), end_vel_.y(), end_vel_.z());
    } else {
        ROS_INFO("End velocity matches target (error < 0.001)");
    }
}

// 修改validateTrajectory函数，确保在轨迹生成后调用
void MinisnapTrajectory::validateTrajectory() {
    if (!trajectory_generated_ || waypoints_.empty() || x_coeffs_.empty()) {
        ROS_WARN("Cannot validate empty trajectory");
        return;
    }
    
    ROS_INFO("Validating trajectory with %zu segments:", x_coeffs_.size());
    
    // 检查起点
    Eigen::Vector3d pos, vel;
    bool success = sampleTrajectory(0.0, pos, vel);
    if (!success) {
        ROS_ERROR("Failed to sample trajectory at t=0.0");
        return;
    }
    
    ROS_INFO("  t=0.00: pos=(%.2f, %.2f, %.2f), expected=(%.2f, %.2f, %.2f)",
             pos.x(), pos.y(), pos.z(),
             waypoints_[0].x(), waypoints_[0].y(), waypoints_[0].z());
    
    // 检查中间点
    double accumulated_time = 0.0;
    for (size_t i = 0; i < segment_times_.size(); ++i) {
        accumulated_time += segment_times_[i];
        success = sampleTrajectory(accumulated_time, pos, vel);
        if (!success) {
            ROS_ERROR("Failed to sample trajectory at t=%.2f", accumulated_time);
            continue;
        }
        
        if (i+1 < waypoints_.size()) {
            ROS_INFO("  t=%.2f: pos=(%.2f, %.2f, %.2f), expected=(%.2f, %.2f, %.2f)",
                     accumulated_time, pos.x(), pos.y(), pos.z(),
                     waypoints_[i+1].x(), waypoints_[i+1].y(), waypoints_[i+1].z());
        }
    }
    
    // 检查终点
    success = sampleTrajectory(total_duration_, pos, vel);
    if (!success) {
        ROS_ERROR("Failed to sample trajectory at t=%.2f", total_duration_);
        return;
    }
    
    ROS_INFO("  t=%.2f: pos=(%.2f, %.2f, %.2f), expected=(%.2f, %.2f, %.2f)",
             total_duration_, pos.x(), pos.y(), pos.z(),
             waypoints_.back().x(), waypoints_.back().y(), waypoints_.back().z());
}

void MinisnapTrajectory::computeCoefficients(const std::vector<Eigen::Vector3d>& waypoints, 
                                           double duration, 
                                           int derivative_order) {
    int n_segments = waypoints.size() - 1;
    int n_coeffs = 2 * derivative_order;  // 每段多项式的系数数量
    
    // 分配分段时间
    segment_times_ = allocateTime(waypoints, duration);
    
    // 打印分段时间信息
    ROS_INFO("Segment times:");
    double total_time = 0.0;
    for (size_t i = 0; i < segment_times_.size(); ++i) {
        total_time += segment_times_[i];
        ROS_INFO("  Segment %zu: %.2f s (%.2f to %.2f)", 
                 i, segment_times_[i], total_time - segment_times_[i], total_time);
    }
    
    // 确保总时间与预期一致
    if (std::abs(total_time - duration) > 1e-3) {
        ROS_WARN("Total segment time (%.2f) differs from requested duration (%.2f)", 
                 total_time, duration);
        // 调整总持续时间以匹配实际分段时间之和
        total_duration_ = total_time;
    }
    
    // 为每个维度创建QP问题
    for (int dim = 0; dim < 3; ++dim) {
        // 设置QP问题的大小
        int n_vars = n_segments * n_coeffs;
        
        // 创建H矩阵（目标函数的二次项）
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
        
        // 创建g向量（目标函数的一次项）
        Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars);
        
        // 创建约束矩阵和向量
        int n_constraints = 2 * derivative_order + (n_segments - 1) * (derivative_order * 2 + 1);
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_vars);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(n_constraints);
        
        // 填充H矩阵（最小化snap）
        for (int i = 0; i < n_segments; ++i) {
            double T = segment_times_[i];
            
            // 对于每段多项式，计算snap的积分
            for (int j = 4; j < n_coeffs; ++j) {
                for (int k = 4; k < n_coeffs; ++k) {
                    double coeff = 0.0;
                    int p1 = j - 4;
                    int p2 = k - 4;
                    coeff = (p1 + 1) * (p1 + 2) * (p1 + 3) * (p1 + 4) * 
                            (p2 + 1) * (p2 + 2) * (p2 + 3) * (p2 + 4) * 
                            std::pow(T, p1 + p2 + 1) / (p1 + p2 + 1);
                    
                    H(i * n_coeffs + j, i * n_coeffs + k) = coeff;
                }
            }
        }
        
        // 填充约束矩阵
        int constraint_idx = 0;
        
        // 起点约束
        for (int d = 0; d < derivative_order; ++d) {
            // 位置约束
            if (d == 0) {
                A(constraint_idx, 0) = 1.0;  // p(0) = waypoint[0]
                b(constraint_idx) = waypoints[0](dim);
            } else {
                // 高阶导数约束为0
                for (int j = d; j < n_coeffs; ++j) {
                    double coeff = 1.0;
                    for (int k = 0; k < d; ++k) {
                        coeff *= (j - k);
                    }
                    A(constraint_idx, j) = coeff * std::pow(0.0, j - d);
                }
                b(constraint_idx) = 0.0;  // 起点的高阶导数为0
            }
            constraint_idx++;
        }
        
        // 终点约束
        for (int d = 0; d < derivative_order; ++d) {
            int last_segment = n_segments - 1;
            double T = segment_times_[last_segment];
            
            // 位置约束
            if (d == 0) {
                for (int j = 0; j < n_coeffs; ++j) {
                    A(constraint_idx, last_segment * n_coeffs + j) = std::pow(T, j);
                }
                b(constraint_idx) = waypoints.back()(dim);
            } else {
                // 高阶导数约束为0
                for (int j = d; j < n_coeffs; ++j) {
                    double coeff = 1.0;
                    for (int k = 0; k < d; ++k) {
                        coeff *= (j - k);
                    }
                    A(constraint_idx, last_segment * n_coeffs + j) = coeff * std::pow(T, j - d);
                }
                b(constraint_idx) = 0.0;  // 终点的高阶导数为0
            }
            constraint_idx++;
        }
        
        // 中间点约束
        for (int i = 0; i < n_segments - 1; ++i) {
            double T = segment_times_[i];
            
            // 连续性约束
            for (int d = 0; d < derivative_order * 2; ++d) {
                if (d == 0) {
                    // 位置约束
                    for (int j = 0; j < n_coeffs; ++j) {
                        A(constraint_idx, i * n_coeffs + j) = std::pow(T, j);
                    }
                    A(constraint_idx, (i + 1) * n_coeffs) = -1.0;  // p_i(T) = p_{i+1}(0)
                    b(constraint_idx) = 0.0;
                } else {
                    // 高阶导数连续性
                    for (int j = d; j < n_coeffs; ++j) {
                        double coeff = 1.0;
                        for (int k = 0; k < d; ++k) {
                            coeff *= (j - k);
                        }
                        A(constraint_idx, i * n_coeffs + j) = coeff * std::pow(T, j - d);
                    }
                    
                    for (int j = d; j < n_coeffs; ++j) {
                        double coeff = 1.0;
                        for (int k = 0; k < d; ++k) {
                            coeff *= (j - k);
                        }
                        A(constraint_idx, (i + 1) * n_coeffs + j) = -coeff * std::pow(0.0, j - d);
                    }
                    b(constraint_idx) = 0.0;
                }
                constraint_idx++;
            }
            
            // 中间点位置约束
            for (int j = 0; j < n_coeffs; ++j) {
                A(constraint_idx, i * n_coeffs + j) = std::pow(T, j);
            }
            b(constraint_idx) = waypoints[i + 1](dim);
            constraint_idx++;
        }
        
        // 求解QP问题
        Eigen::MatrixXd A_t = A.transpose();
        Eigen::MatrixXd H_inv;
        
        // 增强数值稳定性
        Eigen::MatrixXd H_reg = H + Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-8;
        
        if (H_reg.determinant() > 1e-8) {
            H_inv = H_reg.inverse();
        } else {
            // 使用SVD伪逆
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_reg, Eigen::ComputeThinU | Eigen::ComputeThinV);
            double epsilon = 1e-10;  // 奇异值阈值
            Eigen::VectorXd S = svd.singularValues();
            Eigen::VectorXd S_inv = S;
            for (int i = 0; i < S.size(); ++i) {
                if (S(i) > epsilon) {
                    S_inv(i) = 1.0 / S(i);
                } else {
                    S_inv(i) = 0.0;
                }
            }
            H_inv = svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
        }
        
        Eigen::MatrixXd temp = A * H_inv * A_t;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(temp, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd lambda = svd.solve(A * H_inv * g - b);
        Eigen::VectorXd x = H_inv * (g - A_t * lambda);
        
        // 提取系数
        std::vector<std::vector<double>> coeffs(n_segments, std::vector<double>(n_coeffs));
        for (int i = 0; i < n_segments; ++i) {
            for (int j = 0; j < n_coeffs; ++j) {
                coeffs[i][j] = x(i * n_coeffs + j);
            }
        }
        
        // 存储系数
        if (dim == 0) x_coeffs_ = coeffs;
        else if (dim == 1) y_coeffs_ = coeffs;
        else z_coeffs_ = coeffs;
    }
    
    // 检查系数是否合理
    checkCoefficients();
}

void MinisnapTrajectory::checkCoefficients() {
    if (x_coeffs_.empty() || y_coeffs_.empty() || z_coeffs_.empty()) {
        ROS_WARN("Empty coefficient vectors!");
        return;
    }
    
    // 检查第一段多项式在t=0时的值是否等于第一个路径点
    double x0 = evaluatePolynomial(x_coeffs_[0], 0.0, 0);
    double y0 = evaluatePolynomial(y_coeffs_[0], 0.0, 0);
    double z0 = evaluatePolynomial(z_coeffs_[0], 0.0, 0);
    
    double x_error = std::abs(x0 - waypoints_[0].x());
    double y_error = std::abs(y0 - waypoints_[0].y());
    double z_error = std::abs(z0 - waypoints_[0].z());
    
    if (x_error > 1e-3 || y_error > 1e-3 || z_error > 1e-3) {
        ROS_WARN("Start point mismatch: computed=(%.4f, %.4f, %.4f), waypoint=(%.4f, %.4f, %.4f)",
                 x0, y0, z0, waypoints_[0].x(), waypoints_[0].y(), waypoints_[0].z());
    } else {
        ROS_INFO("Start point matches waypoint (error < 0.001)");
    }
    
    // 检查最后一段多项式在t=T时的值是否等于最后一个路径点
    int last_segment = x_coeffs_.size() - 1;
    double T = segment_times_[last_segment];
    double xf = evaluatePolynomial(x_coeffs_[last_segment], T, 0);
    double yf = evaluatePolynomial(y_coeffs_[last_segment], T, 0);
    double zf = evaluatePolynomial(z_coeffs_[last_segment], T, 0);
    
    x_error = std::abs(xf - waypoints_.back().x());
    y_error = std::abs(yf - waypoints_.back().y());
    z_error = std::abs(zf - waypoints_.back().z());
    
    if (x_error > 1e-3 || y_error > 1e-3 || z_error > 1e-3) {
        ROS_WARN("End point mismatch: computed=(%.4f, %.4f, %.4f), waypoint=(%.4f, %.4f, %.4f)",
                 xf, yf, zf, waypoints_.back().x(), waypoints_.back().y(), waypoints_.back().z());
    } else {
        ROS_INFO("End point matches waypoint (error < 0.001)");
    }
}

std::vector<double> MinisnapTrajectory::allocateTime(const std::vector<Eigen::Vector3d>& waypoints, double duration) {
    int n_segments = waypoints.size() - 1;
    std::vector<double> segment_times(n_segments);
    
    // 计算每段的长度
    std::vector<double> segment_lengths(n_segments);
    double total_length = 0.0;
    
    for (int i = 0; i < n_segments; ++i) {
        segment_lengths[i] = (waypoints[i + 1] - waypoints[i]).norm();
        total_length += segment_lengths[i];
    }
    
    // 确保最小持续时间
    for (int i = 0; i < n_segments; ++i) {
        // 分配时间与距离成比例，但确保每段至少有0.5秒
        double segment_duration = std::max(0.5, duration * segment_lengths[i] / total_length);
        segment_times[i] = segment_duration;
    }
    
    // 调整总时间以匹配请求的持续时间
    double sum_time = 0.0;
    for (int i = 0; i < n_segments; ++i) {
        sum_time += segment_times[i];
    }
    
    // 按比例缩放所有段时间
    if (sum_time > 0) {
        double scale = duration / sum_time;
        for (int i = 0; i < n_segments; ++i) {
            segment_times[i] *= scale;
        }
    }
    
    return segment_times;
}

// 修复sampleTrajectory函数中的坐标轴问题
bool MinisnapTrajectory::sampleTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
    if (!trajectory_generated_ || waypoints_.empty() || x_coeffs_.empty() || y_coeffs_.empty() || z_coeffs_.empty()) {
        ROS_ERROR("Cannot sample: Trajectory not generated or invalid!");
        return false;
    }
    
    // 详细的调试输出
    ROS_DEBUG("Sampling trajectory at t=%.3f (total duration: %.3f)", t, total_duration_);
    
    // 特殊处理t=0的情况，确保返回精确的起点
    if (t <= 0.0) {
        pos = waypoints_[0];
        vel = Eigen::Vector3d::Zero();  // 假设起点速度为零
        return true;
    }
    
    // 特殊处理t>=total_duration_的情况，确保返回精确的终点
    if (t >= total_duration_) {
        pos = waypoints_.back();
        vel = Eigen::Vector3d::Zero();  // 假设终点速度为零
        return true;
    }
    
    // 找到当前时间所在的分段
    int segment = 0;
    double segment_time = t;
    double accumulated_time = 0;
    
    for (size_t i = 0; i < segment_times_.size(); ++i) {
        if (t < accumulated_time + segment_times_[i]) {
            segment = i;
            segment_time = t - accumulated_time;
            break;
        }
        accumulated_time += segment_times_[i];
    }
    
    // 确保segment_time不超过当前段的持续时间
    if (segment_time > segment_times_[segment]) {
        ROS_WARN("Segment time (%.3f) exceeds segment duration (%.3f), clamping", 
                segment_time, segment_times_[segment]);
        segment_time = segment_times_[segment];
    }
    
    // 确保segment索引有效
    if (segment >= x_coeffs_.size() || segment >= y_coeffs_.size() || segment >= z_coeffs_.size()) {
        ROS_ERROR("Invalid segment index %d (max: %zu)!", segment, x_coeffs_.size() - 1);
        return false;
    }
    
    // 检查系数向量是否为空
    if (x_coeffs_[segment].empty() || y_coeffs_[segment].empty() || z_coeffs_[segment].empty()) {
        ROS_ERROR("Empty coefficient vectors for segment %d!", segment);
        return false;
    }
    
    // 计算位置
    pos.x() = evaluatePolynomial(x_coeffs_[segment], segment_time, 0);
    pos.y() = evaluatePolynomial(y_coeffs_[segment], segment_time, 0);
    pos.z() = evaluatePolynomial(z_coeffs_[segment], segment_time, 0);
    
    // 计算速度
    vel.x() = evaluatePolynomial(x_coeffs_[segment], segment_time, 1);
    vel.y() = evaluatePolynomial(y_coeffs_[segment], segment_time, 1);
    vel.z() = evaluatePolynomial(z_coeffs_[segment], segment_time, 1);
    
    ROS_DEBUG("Sampled position: (%.3f, %.3f, %.3f), velocity: (%.3f, %.3f, %.3f)", 
             pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z());
    
    return true;
}

double MinisnapTrajectory::evaluatePolynomial(const std::vector<double>& coeffs, double t, int derivative) {
    if (coeffs.empty()) {
        ROS_ERROR("Error: Empty coefficient vector!");
        return 0.0;
    }
    
    double result = 0.0;
    
    if (derivative == 0) {
        // 计算多项式值
        for (size_t i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * std::pow(t, i);
        }
    } else {
        // 计算多项式导数
        for (size_t i = derivative; i < coeffs.size(); ++i) {
            double coeff = coeffs[i];
            for (int j = 0; j < derivative; ++j) {
                coeff *= (i - j);
            }
            result += coeff * std::pow(t, i - derivative);
        }
    }
    
    return result;
}

double MinisnapTrajectory::getTotalDuration() const {
    // 如果轨迹未生成，返回0
    if (!trajectory_generated_) {
        return 0.0;
    }
    
    // 返回存储的总持续时间
    return total_duration_;
}

bool MinisnapTrajectory::isTrajectoryGenerated() const {
    return trajectory_generated_;
}