// MPCController.cpp (重构实现)
#include "offboard_pkg/MPCController.h"

MPCController::MPCController() : solver_initialized_(false), thr2acc_(15.0) {
    // 设置权重矩阵
    params_.Q_p = Eigen::Matrix<double, 3, 3>::Identity() * 200;
    params_.Q_p(2, 2) = 500;
    params_.Q_v = Eigen::Matrix<double, 3, 3>::Identity() * 50;
    params_.R = Eigen::Matrix<double, 4, 4>::Identity() * 10;
    params_.R(0, 0) = 200.0;
    params_.R(3, 3) = 100.0; // yaw权重 - 大幅增加，抑制偏航变化
    params_.Q_obs = 400; //将避障项加入成本函数
    // 避障参数
    params_.safe_distance_obstacle = 0.25;  // 安全距离，将避障项加入约束
}

bool MPCController::initialize() {
    try {
        // 定义优化变量：U = [thrust, roll, pitch, yaw] for each time step
        casadi::SX U = casadi::SX::sym("U", 4, params_.horizon);
        
        // 参数：初始状态 + 参考轨迹 + thr2acc
        casadi::SX X0 = casadi::SX::sym("X0", 6);  // [pos, vel]
        casadi::SX X_ref = casadi::SX::sym("X_ref", 6 * params_.horizon);  // 参考轨迹
        casadi::SX thr2acc_param = casadi::SX::sym("thr2acc", 1);  // 油门转化比值（参数）
        // 观测扰动：[dx, dy, dz]
        casadi::SX observed_disturbance = casadi::SX::sym("observed_disturbance", 3); // 观测的扰动（参数）

        // 障碍物信息参数
        int obs_num = 3;
        casadi::SX obstacles_pos = casadi::SX::sym("obstacles_pos", 4 * obs_num);  // [x,y,z,radius]
        casadi::SX num_obstacles = casadi::SX::sym("num_obstacles", 1);

        // 邻居信息参数
        int nei_num = 2;
        casadi::SX neighbors_pos = casadi::SX::sym("neighbors_pos", 3 * nei_num);
        casadi::SX neighbors_vel = casadi::SX::sym("neighbors_vel", 3 * nei_num);
        casadi::SX num_neighbors = casadi::SX::sym("num_neighbors", 1);
        
        // 转换权重矩阵
        casadi::SX Q_p_casadi = eigenToCasadi(params_.Q_p);
        casadi::SX Q_v_casadi = eigenToCasadi(params_.Q_v);
        casadi::SX R_casadi = eigenToCasadi(params_.R);
        
        // 目标函数
        casadi::SX obj = 0;
        std::vector<casadi::SX> constraints;
        
        // 当前状态
        casadi::SX x_current = X0;
        
        for (int k = 0; k < params_.horizon; ++k) {
            // 当前控制输入
            casadi::SX u_k = U(casadi::Slice(), k);
            
            // 动力学传播
            casadi::SX x_next = createQuadrotorModel(x_current, u_k, thr2acc_param, observed_disturbance);
            
            // 参考状态
            casadi::SX x_ref_k = X_ref(casadi::Slice(k*6, (k+1)*6));
            
            // 状态误差
            casadi::SX pos_error = x_current(casadi::Slice(0,3)) - x_ref_k(casadi::Slice(0,3));
            casadi::SX vel_error = x_current(casadi::Slice(3,6)) - x_ref_k(casadi::Slice(3,6));
            
            // 位置和速度代价
            obj += casadi::SX::mtimes({pos_error.T(), Q_p_casadi, pos_error});
            obj += casadi::SX::mtimes({vel_error.T(), Q_v_casadi, vel_error});
            
            // 控制代价（相对于悬停推力）
            casadi::SX u_hover = casadi::SX::vertcat({
                params_.gravity / thr2acc_param,  // 悬停推力
                0.0, 0.0, 0.0  // 零姿态角
            });
            casadi::SX u_error = u_k - u_hover;
            obj += casadi::SX::mtimes({u_error.T(), R_casadi, u_error});

            // 平滑性代价
            casadi::SX smoothness_cost = 0;
            for (int k = 1; k < params_.horizon; ++k) {
                casadi::SX u_diff = U(casadi::Slice(), k) - U(casadi::Slice(), k-1);
                smoothness_cost += 100.0 * casadi::SX::dot(u_diff, u_diff); //200
            }
            obj += smoothness_cost;
            
            // // 避障代价（势场），不使用时可以注释
            casadi::SX obstacle_cost = 0;
            for (int obs = 0; obs < obs_num; ++obs) {
                casadi::SX obs_pos = obstacles_pos(casadi::Slice(obs*4, obs*4+2));
                casadi::SX obs_radius = obstacles_pos(obs*4+3);

                // 当前位置和速度
                casadi::SX current_pos = x_current(casadi::Slice(0,2));
                casadi::SX current_vel = x_current(casadi::Slice(3,5)); // 假设速度2-3
                
                casadi::SX dist_to_obs = casadi::SX::norm_2(x_current(casadi::Slice(0,2)) - obs_pos);
                casadi::SX safe_dist = obs_radius + params_.safe_distance_obstacle;

                // 计算相对位置向量（从障碍物指向无人机）
                casadi::SX relative_pos = current_pos - obs_pos;
                casadi::SX relative_pos_norm = casadi::SX::norm_2(relative_pos);
                casadi::SX relative_pos_unit = relative_pos / (relative_pos_norm + 1e-6); // 避免除零

                // 计算速度方向单位向量
                casadi::SX vel_norm = casadi::SX::norm_2(current_vel);
                casadi::SX vel_unit = current_vel / (vel_norm + 1e-6); // 避免除零

                // 计算速度与相对位置的夹角余弦值
                casadi::SX cos_angle = casadi::SX::dot(vel_unit, relative_pos_unit); 
                
                // 方向性因子：当速度方向远离障碍物时减小势场影响
                // cos_angle > 0 表示远离障碍物，cos_angle < 0 表示靠近障碍物
                casadi::SX direction_factor = casadi::SX::if_else(
                    cos_angle > 0.0,
                    // 远离障碍物时，根据角度和距离动态调整
                    1.0 - 0.9 * cos_angle * casadi::SX::tanh(dist_to_obs / safe_dist - 1.0),
                    // 靠近障碍物时，保持较强的势场
                    1.0 + 0.3 * casadi::SX::abs(cos_angle)
                );
                
                // 影响范围
                casadi::SX influence_range = safe_dist * 2.0;  // 扩大影响范围
                casadi::SX danger_zone = safe_dist * 1.2;      // 危险区域
                
                casadi::SX obs_exists = casadi::SX::if_else(obs < num_obstacles, 1.0, 0.0);

                // 使用更平滑的势场函数
                casadi::SX potential = casadi::SX::if_else(
                    dist_to_obs < influence_range,
                    casadi::SX::if_else(
                        dist_to_obs < danger_zone,
                        // 危险区域 - 使用更平滑的函数
                        params_.Q_obs * direction_factor * 
                        casadi::SX::exp(-dist_to_obs / safe_dist) / (dist_to_obs + safe_dist * 0.1),
                        // 警告区域 - 使用指数衰减
                        params_.Q_obs * direction_factor * 0.2 * 
                        casadi::SX::exp(-3.0 * (dist_to_obs - danger_zone) / (influence_range - danger_zone))
                    ),
                    0.0
                );
                obstacle_cost += obs_exists * potential;
            }
            obj += obstacle_cost;
            
            // 邻居避碰代价
            casadi::SX neighbor_cost = 0;
            casadi::SX current_pos = x_current(casadi::Slice(0,3));
            casadi::SX current_vel = x_current(casadi::Slice(3,6));
            
            for (int n = 0; n < nei_num; ++n) {
                // 邻居位置和速度
                casadi::SX neighbor_pos = neighbors_pos(casadi::Slice(n*3, n*3+3));
                casadi::SX neighbor_vel = neighbors_vel(casadi::Slice(n*3, n*3+3));
                
                // 预测邻居未来位置（假设匀速运动）
                double future_time = k * params_.dt;
                casadi::SX predicted_neighbor_pos = neighbor_pos + neighbor_vel * future_time;
                
                // 计算距离
                casadi::SX dist_to_neighbor = casadi::SX::norm_2(current_pos - predicted_neighbor_pos);
                
                // 邻居存在标志
                casadi::SX neighbor_exists = casadi::SX::if_else(n < num_neighbors, 1.0, 0.0);
                casadi::SX dynamic_safe_dist = params_.safe_distance_neighbor;
                // 势场函数：距离越近，代价越高
                casadi::SX potential = casadi::SX::if_else(
                        dist_to_neighbor < dynamic_safe_dist,
                        // 危险区域：使用高代价
                        params_.Q_neighbor * casadi::SX::exp(-dist_to_neighbor / dynamic_safe_dist) / 
                        (dist_to_neighbor + dynamic_safe_dist * 0.1),
                        // 警告区域：使用较低代价
                        0
                    )
                ;
                
                neighbor_cost += neighbor_exists * potential;
            }
            
            // obj += neighbor_cost;// 暂未考虑
            
            // 控制输入约束
            constraints.push_back(u_k(0));  // 推力约束
            constraints.push_back(u_k(1));  // roll约束
            constraints.push_back(u_k(2));  // pitch约束
            constraints.push_back(u_k(3));  // yaw约束
            
            // 更新状态
            x_current = x_next;
        }
        
        // 终端代价
        casadi::SX x_ref_final = X_ref(casadi::Slice((params_.horizon-1)*6, params_.horizon*6));
        casadi::SX pos_error_final = x_current(casadi::Slice(0,3)) - x_ref_final(casadi::Slice(0,3));
        casadi::SX vel_error_final = x_current(casadi::Slice(3,6)) - x_ref_final(casadi::Slice(3,6));
        obj += 2.0 * casadi::SX::mtimes({pos_error_final.T(), Q_p_casadi, pos_error_final});
        obj += 2.0 * casadi::SX::mtimes({vel_error_final.T(), Q_v_casadi, vel_error_final});
        
        // 构建NLP
        casadi::SXDict nlp = {
            {"x", casadi::SX::reshape(U, -1, 1)},
            {"p", casadi::SX::vertcat({X0, X_ref, thr2acc_param, obstacles_pos, num_obstacles, neighbors_pos, neighbors_vel, num_neighbors, observed_disturbance})},
            {"f", obj},
            {"g", casadi::SX::vertcat(constraints)}
        };
        
        // 求解器选项
        casadi::Dict opts;
        opts["ipopt.tol"] = 1e-3;
        opts["ipopt.max_iter"] = 80;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.warm_start_init_point"] = "yes";
        
        solver_ = casadi::nlpsol("mpc_solver", "ipopt", nlp, opts);
        solver_initialized_ = true;
        
        ROS_INFO("MPC solver initialized successfully");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize MPC solver: %s", e.what());
        return false;
    }
}

MPCController::ControlOutput MPCController::solve(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& current_vel,
    const std::vector<Eigen::Vector3d>& ref_positions,
    const std::vector<Eigen::Vector3d>& ref_velocities,
    double thr2acc,
    const std::vector<Obstacle>& obstacles,
    const std::vector<Eigen::Vector3d>& neighbor_positions,
    const std::vector<Eigen::Vector3d>& neighbor_velocities,
    NonlinearESO &observer) {

    // 更新当前状态
    ControlOutput output;
    if (!solver_initialized_) {
        ROS_ERROR("MPC solver not initialized");
        return output;
    }
    
    try {
        // 获取当前扰动估计
        std::vector<double> disturbance(3);
        observer.getDisturbanceEstimate(disturbance.data());
        // 构建初始状态
        std::vector<double> x0 = {
            current_pos.x(), current_pos.y(), current_pos.z(),
            current_vel.x(), current_vel.y(), current_vel.z()
        };
        
        // 构建参考轨迹
        std::vector<double> x_ref;
        for (int k = 0; k < params_.horizon; ++k) {
            int idx = std::min(k, (int)ref_positions.size() - 1);
            x_ref.push_back(ref_positions[idx].x());
            x_ref.push_back(ref_positions[idx].y());
            x_ref.push_back(ref_positions[idx].z());
            x_ref.push_back(ref_velocities[idx].x());
            x_ref.push_back(ref_velocities[idx].y());
            x_ref.push_back(ref_velocities[idx].z());
        }

        // 构建障碍物信息
        std::vector<double> obstacles_pos;
        int num_obstacles = 3;
        
        for (int i = 0; i < num_obstacles; ++i) {
            obstacles_pos.push_back(obstacles[i].position.x());
            obstacles_pos.push_back(obstacles[i].position.y());
            obstacles_pos.push_back(obstacles[i].position.z());
            obstacles_pos.push_back(obstacles[i].radius);
        }

        // 构建邻居信息
        std::vector<double> neighbors_pos_vec, neighbors_vel_vec;
        const int FIXED_NEI_NUM = 2;
        int actual_nei_num = std::min({(int)neighbor_positions.size(), 
                                      (int)neighbor_velocities.size(), 
                                      FIXED_NEI_NUM});
        
        for (int i = 0; i < FIXED_NEI_NUM; ++i) {
            if (i < neighbor_positions.size() && i < neighbor_velocities.size()) {
                // 使用实际邻居
                neighbors_pos_vec.push_back(neighbor_positions[i].x());
                neighbors_pos_vec.push_back(neighbor_positions[i].y());
                neighbors_pos_vec.push_back(neighbor_positions[i].z());
                
                neighbors_vel_vec.push_back(neighbor_velocities[i].x());
                neighbors_vel_vec.push_back(neighbor_velocities[i].y());
                neighbors_vel_vec.push_back(neighbor_velocities[i].z());
            } else {
                // 填充虚拟邻居（远离当前位置）
                neighbors_pos_vec.push_back(1000.0); // x
                neighbors_pos_vec.push_back(1000.0); // y
                neighbors_pos_vec.push_back(1000.0); // z
                neighbors_vel_vec.push_back(0.0);    // vx
                neighbors_vel_vec.push_back(0.0);    // vy
                neighbors_vel_vec.push_back(0.0);    // vz
            }
        }
        
        // 构建参数向量
        std::vector<double> params_vec = x0;
        params_vec.insert(params_vec.end(), x_ref.begin(), x_ref.end());
        params_vec.push_back(thr2acc);
        params_vec.insert(params_vec.end(), obstacles_pos.begin(), obstacles_pos.end());
        params_vec.push_back((double)num_obstacles);
        params_vec.insert(params_vec.end(), neighbors_pos_vec.begin(), neighbors_pos_vec.end());
        params_vec.insert(params_vec.end(), neighbors_vel_vec.begin(), neighbors_vel_vec.end());
        params_vec.push_back((double)FIXED_NEI_NUM);
        disturbance[0] = 0;
        disturbance[1] = 0;
        disturbance[2] = 0;
        params_vec.insert(params_vec.end(), disturbance.begin(), disturbance.end());

        // 初始猜测（热启动或简单初始化）
        std::vector<double> x_init;
        if (last_solution_.size() == 4 * params_.horizon) {
            // 热启动：使用上次解的shifted版本
            x_init.assign(last_solution_.begin() + 4, last_solution_.end());
            // 最后一步重复
            for (int i = 0; i < 4; ++i) {
                x_init.push_back(last_solution_[last_solution_.size() - 4 + i]);
            }
        } else {
            // 冷启动：悬停初始化
            double hover_thrust = params_.gravity / thr2acc;
            for (int k = 0; k < params_.horizon; ++k) {
                x_init.push_back(hover_thrust);  // thrust
                x_init.push_back(0.0);           // roll
                x_init.push_back(0.0);           // pitch
                x_init.push_back(0.0);           // yaw
            }
        }
        
        // 约束边界
        std::vector<double> lbg, ubg;
        for (int k = 0; k < params_.horizon; ++k) {
            lbg.push_back(params_.thrust_min);      // thrust lower
            ubg.push_back(params_.thrust_max);      // thrust upper
            lbg.push_back(-params_.angle_max);      // roll lower
            ubg.push_back(params_.angle_max);       // roll upper
            lbg.push_back(-params_.angle_max);      // pitch lower
            ubg.push_back(params_.angle_max);       // pitch upper
            lbg.push_back(-M_PI);                   // yaw lower
            ubg.push_back(M_PI);                    // yaw upper
        }
        
        // 变量边界
        std::vector<double> lbx, ubx;
        for (int k = 0; k < params_.horizon; ++k) {
            lbx.push_back(params_.thrust_min);      // thrust
            ubx.push_back(params_.thrust_max);
            lbx.push_back(-params_.angle_max);      // roll
            ubx.push_back(params_.angle_max);
            lbx.push_back(-params_.angle_max);      // pitch
            ubx.push_back(params_.angle_max);
            lbx.push_back(-M_PI);                   // yaw
            ubx.push_back(M_PI);
        }
        
        // 求解
        casadi::DMDict arg = {
            {"x0", x_init},
            {"p", params_vec},
            {"lbx", lbx},
            {"ubx", ubx},
            {"lbg", lbg},
            {"ubg", ubg}
        };
        
        casadi::DMDict result = solver_(arg);
        
        // 提取解
        std::vector<double> solution = result.at("x").get_elements();
        last_solution_ = solution;

        // 输出第一步控制
        output.thrust = solution[0];
        output.euler_angles = Eigen::Vector3d(solution[1], solution[2], solution[3]);
        output.success = true;
        // 更新观测器
        observer.update(current_pos, output.thrust, output.euler_angles);
        
        return output;
        
    } catch (const std::exception& e) {
        ROS_WARN("MPC solve failed: %s", e.what());
        // 失败时返回悬停控制
        output.thrust = params_.gravity / thr2acc;
        output.euler_angles = Eigen::Vector3d::Zero();
        output.success = false;
        return output;
    }
}

// 在cpp文件中修改实现：
casadi::SX MPCController::createQuadrotorModel(const casadi::SX& x, const casadi::SX& u, const casadi::SX& thr2acc, const casadi::SX &d) {
    casadi::SX pos = x(casadi::Slice(0, 3));
    casadi::SX vel = x(casadi::Slice(3, 6));
    
    casadi::SX thrust = u(0);
    casadi::SX roll = u(1);
    casadi::SX pitch = u(2);
    casadi::SX yaw = u(3);

    // 旋转矩阵
    casadi::SX R = eulerToRotationMatrix(roll, pitch, yaw);
    
    // 使用传入的thr2acc参数
    casadi::SX thrust_acc = thrust * thr2acc;  // 推力转换为加速度
    casadi::SX F_body = casadi::SX::vertcat({0, 0, thrust_acc});
    
    // 转换到惯性系
    casadi::SX F_inertial = casadi::SX::mtimes(R, F_body) + d;
    
    // 重力
    casadi::SX gravity = casadi::SX::vertcat({0, 0, -params_.gravity});
    
    // 直接相加，因为F_inertial已经是加速度单位
    casadi::SX acc = F_inertial + gravity;
    
    // 状态导数
    casadi::SX pos_dot = vel;
    casadi::SX vel_dot = acc;
    
    // 离散化
    casadi::SX x_next = casadi::SX::vertcat({
        pos + params_.dt * pos_dot,
        vel + params_.dt * vel_dot
    });
    
    return x_next;
}

casadi::SX MPCController::eulerToRotationMatrix(const casadi::SX& roll, const casadi::SX& pitch, const casadi::SX& yaw) {
    // ZYX欧拉角旋转矩阵
    casadi::SX cr = casadi::SX::cos(roll);
    casadi::SX sr = casadi::SX::sin(roll);
    casadi::SX cp = casadi::SX::cos(pitch);
    casadi::SX sp = casadi::SX::sin(pitch);
    casadi::SX cy = casadi::SX::cos(yaw);
    casadi::SX sy = casadi::SX::sin(yaw);
    
    casadi::SX R = casadi::SX::zeros(3, 3);
    
    R(0,0) = cp * cy;
    R(0,1) = sr * sp * cy - cr * sy;
    R(0,2) = cr * sp * cy + sr * sy;
    
    R(1,0) = cp * sy;
    R(1,1) = sr * sp * sy + cr * cy;
    R(1,2) = cr * sp * sy - sr * cy;
    
    R(2,0) = -sp;
    R(2,1) = sr * cp;
    R(2,2) = cr * cp;
    
    return R;
}

template<typename Derived>
casadi::SX MPCController::eigenToCasadi(const Eigen::MatrixBase<Derived>& mat) {
    casadi::SX result = casadi::SX::zeros(mat.rows(), mat.cols());
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            result(i, j) = mat(i, j);
        }
    }
    return result;
}
