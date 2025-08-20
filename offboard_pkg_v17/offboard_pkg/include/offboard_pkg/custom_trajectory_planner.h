// custom_trajectory_planner.h
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>
#include <Eigen/Dense>
#include "offboard_pkg/astar_planner.h"
#include "offboard_pkg/minisnap_trajectory.h"
#include <map>
#include <nav_msgs/Path.h>


class CustomTrajectoryPlanner {
public:
    CustomTrajectoryPlanner();
    ~CustomTrajectoryPlanner();
    
    // 生成轨迹（直接连接起点和终点）
    bool generateTrajectory(const Eigen::Vector3d& start_pos, 
                           const Eigen::Vector3d& end_pos,
                           double duration);
    
    // 使用APF方法生成避障路径（保留原有方法，以便比较）
    bool generatePathWithAPF(const Eigen::Vector3d& start_pos, 
                            const Eigen::Vector3d& end_pos,
                            double duration,
                            double step_size);
    
    // 使用A*和Minisnap生成避障轨迹
    bool generatePathWithAStarAndMinisnap(const Eigen::Vector3d& start_pos,
                                         const Eigen::Vector3d& end_pos,
                                         double duration,
                                         double grid_resolution = 0.2);
    
    // 采样轨迹点
    bool sampleTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel);
    
    // 设置障碍物
    void setObstacles(const std::vector<Obstacle>& obstacles);
    
    // 设置APF参数
    void setAPFParams(double attractive_gain, double repulsive_gain, 
                     double safe_distance, double goal_threshold);
    
    // 获取起点和终点
    Eigen::Vector3d getStartPosition() const { return start_pos_; }
    Eigen::Vector3d getEndPosition() const { return end_pos_; }
    
    // 获取生成的路径点
    const std::vector<Eigen::Vector3d>& getPath() const;

    // 发布A*规划的路径点
    // 用于持续发布路径点的定时器和发布器
    std::map<std::string, ros::Publisher> path_point_pubs_;
    std::map<std::string, ros::Timer> path_publish_timers_;
    // 开始发布A*规划的路径点
    void startPublishingPathPoints(ros::NodeHandle& nh, const std::string& uav_name);

    //测试
    void setFormationOffsets(const std::map<std::string, Eigen::Vector3d>& offsets);
    
    // 使用考虑编队约束的A*和Minisnap生成避障轨迹
    bool generatePathWithFormationConstraints(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& end_pos,
        double duration,
        double grid_resolution);
    // 编队偏移量
    std::map<std::string, Eigen::Vector3d> formation_offsets_;

    // 预规划
    bool generateFollowerTrajectory(
        const std::vector<Eigen::Vector3d>& leader_path,  // 修改为向量类型
        const Eigen::Vector3d& offset,
        double duration);
    

    void publishPathAsRosMsg(const ros::Publisher& pub) const;
    static std::vector<Eigen::Vector3d> rosPathMsgToEigen(const nav_msgs::Path& path_msg);
    bool isTrajectoryGenerated() const;    
    // 获取轨迹总持续时间
    double getDuration() const { return duration_; }
    bool generatePathWithInitialVelocity(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, const Eigen::Vector3d& start_vel, double duration) ;

private:
    // 计算五次多项式系数
    void calculateQuinticPolynomial(double start, double end, double duration,
                                  std::vector<double>& coefficients);
    
    // 带约束的五次多项式系数计算
    std::vector<double> calculateQuinticPolynomialWithConstraints(
        double p0, double p1, double v0, double v1, double a0, double a1, double T);
    
    // 计算引力
    Eigen::Vector3d calculateAttractiveForce(const Eigen::Vector3d& position, 
                                           const Eigen::Vector3d& goal);
    
    // 计算斥力
    Eigen::Vector3d calculateRepulsiveForce(const Eigen::Vector3d& position);
    
    // 计算总力
    Eigen::Vector3d calculateTotalForce(const Eigen::Vector3d& position, 
                                      const Eigen::Vector3d& goal);
    
    // 处理局部最小值
    Eigen::Vector3d escapeLocalMinimum(const Eigen::Vector3d& position, 
                                     const Eigen::Vector3d& goal);
    
    // 平滑路径
    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& raw_path);
    
    // 分段多项式拟合
    bool fitSegmentedTrajectory(const std::vector<Eigen::Vector3d>& path, double duration);
    
    // 将路径点拟合为多项式轨迹
    bool fitPolynomialTrajectory(const std::vector<Eigen::Vector3d>& path, double duration);
    
    // 采样分段轨迹
    bool sampleSegmentedTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel);

    

    

private:
    Eigen::Vector3d start_pos_;  // 起点
    Eigen::Vector3d end_pos_;    // 终点
    double duration_;            // 轨迹持续时间
    bool trajectory_generated_;  // 轨迹是否已生成
    bool reached_goal_;          // 是否已到达目标
    
    // 多项式系数
    std::vector<double> x_coeffs_;
    std::vector<double> y_coeffs_;
    std::vector<double> z_coeffs_;
    
    // 分段轨迹
    std::vector<std::vector<double>> segment_coeffs_x_;
    std::vector<std::vector<double>> segment_coeffs_y_;
    std::vector<std::vector<double>> segment_coeffs_z_;
    std::vector<double> segment_times_;
    bool use_segmented_trajectory_;
    
    // APF参数
    double attractive_gain_;
    double repulsive_gain_;
    double safe_distance_;
    double goal_threshold_;
    
    // 障碍物
    std::vector<Obstacle> obstacles_;
    
    // 路径点
    std::vector<Eigen::Vector3d> path_;
    
    // A*规划器
    AStarPlanner* astar_planner_;
    
    // Minisnap轨迹生成器
    MinisnapTrajectory* minisnap_trajectory_;
};
