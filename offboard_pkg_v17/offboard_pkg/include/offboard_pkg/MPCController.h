// MPCController.h (新增)
#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "offboard_pkg/obstacle.h"
#include "offboard_pkg/NonlinearESO.hpp"
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <uav_utils/utils.h> 
#include "offboard_pkg/input.h"

class MPCController {
public:
    Obstacle obstacles_;
    struct MPCParams {
        int horizon = 15;
        double dt = 0.05;
        double mass = 1.55;
        double gravity = 9.81;
        
        // 权重矩阵
        Eigen::Matrix3d Q_p = Eigen::Matrix3d::Identity() * 50.0;  // 位置权重
        Eigen::Matrix3d Q_v = Eigen::Matrix3d::Identity() * 1.0;   // 速度权重
        Eigen::Matrix4d R = Eigen::Matrix4d::Identity() * 1.0;     // 控制权重
        double Q_obs = 1000.0; 
        double safe_distance_obstacle = 0.5;  // 与障碍物的安全距离

        // 约束
        double thrust_min = 0.1;
        double thrust_max = 0.8;
        double angle_max = 0.15;  // 最大倾斜角(弧度)

        double Q_neighbor = 200.0;           // 邻居避碰权重
        double safe_distance_neighbor = 0.4; // 邻居间安全距离
        double influence_range_neighbor = 1.0; // 邻居影响范围
    };

    struct ControlOutput {
        double thrust;
        Eigen::Vector3d euler_angles;  // [roll, pitch, yaw]
        bool success = false;
    };

    MPCController();
    ~MPCController() = default;

    bool initialize();
    ControlOutput solve(const Eigen::Vector3d& current_pos,
                       const Eigen::Vector3d& current_vel,
                       const std::vector<Eigen::Vector3d>& ref_positions,
                       const std::vector<Eigen::Vector3d>& ref_velocities,
                       double thr2acc,
                       const std::vector<Obstacle>& obstacles,
                       const std::vector<Eigen::Vector3d>& neighbor_positions,
                       const std::vector<Eigen::Vector3d>& neighbor_velocities,
                       NonlinearESO &observer);

    void updateThr2Acc(double new_thr2acc) { thr2acc_ = new_thr2acc; }
    

private:
    MPCParams params_;
    casadi::Function solver_;
    bool solver_initialized_;
    double thr2acc_;
    
    // 上一次的解，用于热启动
    std::vector<double> last_solution_;

    casadi::Function cost_analyzer_;  // 成本分析函数
    
    // 辅助函数
    casadi::SX createQuadrotorModel(const casadi::SX& x, const casadi::SX& u, const casadi::SX& thr2acc, const casadi::SX &d);

    casadi::SX eulerToRotationMatrix(const casadi::SX& roll, const casadi::SX& pitch, const casadi::SX& yaw);
    template<typename Derived>
    casadi::SX eigenToCasadi(const Eigen::MatrixBase<Derived>& mat);
};

#endif
