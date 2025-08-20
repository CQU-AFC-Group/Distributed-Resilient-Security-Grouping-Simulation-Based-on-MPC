// minisnap_trajectory.h

#ifndef MINISNAP_TRAJECTORY_H
#define MINISNAP_TRAJECTORY_H

#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

class MinisnapTrajectory {
public:
    MinisnapTrajectory();
    ~MinisnapTrajectory();
    
    /**
     * 生成最小化snap的轨迹
     * @param waypoints 路径点列表
     * @param duration 总持续时间
     * @param derivative_order 导数阶数，通常为4（最小化snap）
     * @return 是否成功生成轨迹
     */
    bool generateTrajectory(const std::vector<Eigen::Vector3d>& waypoints, 
                           double duration = 10.0, 
                           int derivative_order = 4);
    
    /**
     * 在指定时间采样轨迹
     * @param t 时间
     * @param pos 输出位置
     * @param vel 输出速度
     * @return 是否成功采样
     */
    bool sampleTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel);
    
    /**
     * 获取轨迹总持续时间
     * @return 总持续时间
     */
    double getTotalDuration() const;

    bool isTrajectoryGenerated() const;

    bool generateTrajectoryWithInitialVelocity(
        const std::vector<Eigen::Vector3d>& waypoints,
        const Eigen::Vector3d& start_vel,
        const Eigen::Vector3d& end_vel,
        double duration,
        int derivative_order = 4);

    // 新增成员
    Eigen::Vector3d start_vel_;
    Eigen::Vector3d end_vel_;
    
    // 新增方法
    void computeCoefficientsWithVelocity(
        const std::vector<Eigen::Vector3d>& waypoints,
        const Eigen::Vector3d& start_vel,
        const Eigen::Vector3d& end_vel,
        double duration,
        int derivative_order);
        
    void checkCoefficientsWithVelocity();
    
private:
    /**
     * 计算多项式系数
     * @param waypoints 路径点列表
     * @param duration 总持续时间
     * @param derivative_order 导数阶数
     */
    void computeCoefficients(const std::vector<Eigen::Vector3d>& waypoints, 
                            double duration, 
                            int derivative_order);
    
    /**
     * 分配分段时间
     * @param waypoints 路径点列表
     * @param duration 总持续时间
     * @return 分段时间列表
     */
    std::vector<double> allocateTime(const std::vector<Eigen::Vector3d>& waypoints, 
                                    double duration);
    
    /**
     * 计算多项式在指定时间的值或导数
     * @param coeffs 多项式系数
     * @param t 时间
     * @param derivative 导数阶数（0表示位置，1表示速度）
     * @return 计算结果
     */
    double evaluatePolynomial(const std::vector<double>& coeffs, 
                             double t, 
                             int derivative);
    
    /**
     * 验证生成的轨迹
     * 在关键时间点采样轨迹并与预期路径点比较
     */
    void validateTrajectory();
    
    /**
     * 检查计算的多项式系数是否合理
     * 验证起点和终点是否与路径点匹配
     */
    void checkCoefficients();
    
    // 存储路径点
    std::vector<Eigen::Vector3d> waypoints_;
    
    // 存储分段时间
    std::vector<double> segment_times_;
    
    // 存储多项式系数（每个维度每段一组系数）
    std::vector<std::vector<double>> x_coeffs_;
    std::vector<std::vector<double>> y_coeffs_;
    std::vector<std::vector<double>> z_coeffs_;
    
    // 轨迹总持续时间
    double total_duration_;
    
    // 轨迹是否已生成
    bool trajectory_generated_;
};

#endif // MINISNAP_TRAJECTORY_H
