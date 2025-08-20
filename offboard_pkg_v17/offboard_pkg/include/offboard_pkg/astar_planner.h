// astar_planner.h
#pragma once

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <Eigen/Dense>
#include <map>
#include "offboard_pkg/obstacle.h"



struct Node {
    Eigen::Vector3i index;  // 栅格索引
    double g_cost;         // 从起点到当前节点的代价
    double h_cost;         // 从当前节点到终点的启发式估计
    double f_cost;         // f = g + h
    Node* parent;          // 父节点

    // 构造函数
    Node(const Eigen::Vector3i& idx, double g = 0.0, double h = 0.0, Node* p = nullptr)
        : index(idx), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    // 比较运算符（用于优先队列）
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

// 自定义哈希函数，用于unordered_set/map
struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& v) const {
        return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y()) ^ std::hash<int>()(v.z());
    }
};

// 自定义相等函数，用于unordered_set/map
struct Vector3iEqual {
    bool operator()(const Eigen::Vector3i& v1, const Eigen::Vector3i& v2) const {
        return v1.x() == v2.x() && v1.y() == v2.y() && v1.z() == v2.z();
    }
};

class AStarPlanner {
public:
    // Obstacle obstacles_A;
    AStarPlanner(double resolution = 0.2);
    ~AStarPlanner();

    // 设置障碍物
    void setObstacles(const std::vector<Obstacle>& obstacles);
    
    // 设置规划空间范围
    void setPlanningBounds(const Eigen::Vector3d& min_bounds, const Eigen::Vector3d& max_bounds);
    
    // 执行A*规划
    bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& path);

    //测试：
    // 设置编队偏移量
    void setFormationOffsets(const std::map<std::string, Eigen::Vector3d>& offsets);
    
    // 考虑编队约束的规划方法
    bool planWithFormationConstraints(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, 
                                     std::vector<Eigen::Vector3d>& path);

     // 编队偏移量
     std::map<std::string, Eigen::Vector3d> formation_offsets_;
    
     // 评估编队约束
     double evaluateFormationConstraint(const Eigen::Vector3d& pos) const;
     
     // 优化路径以满足编队约束
     std::vector<Eigen::Vector3d> optimizePathForFormation(
         const std::vector<Eigen::Vector3d>& path) const;

    // 检查位置是否碰撞
    bool isCollisionFree(const Eigen::Vector3d& pos) const;

    
private:
    // 连续坐标转换为栅格索引
    Eigen::Vector3i posToIndex(const Eigen::Vector3d& pos) const;
    
    // 栅格索引转换为连续坐标
    Eigen::Vector3d indexToPos(const Eigen::Vector3i& index) const;
    
    // 检查索引是否有效
    bool isIndexValid(const Eigen::Vector3i& index) const;
    
    
    
    // 计算启发式函数值（欧氏距离）
    double calculateHeuristic(const Eigen::Vector3i& index, const Eigen::Vector3i& goal_index) const;
    
    // 获取节点的邻居节点
    std::vector<Eigen::Vector3i> getNeighbors(const Eigen::Vector3i& index) const;
    
    // 从终点回溯构建路径
    // 三维
    // std::vector<Eigen::Vector3d> reconstructPath(Node* goal_node) const;
    // 二维
    std::vector<Eigen::Vector3d> reconstructPath(Node* goal_node, double fixed_z) const;

    // 对路径进行简化（去除冗余点）
    std::vector<Eigen::Vector3d> simplifyPath(const std::vector<Eigen::Vector3d>& path) const;

private:
    double resolution_;                 // 栅格分辨率
    Eigen::Vector3d min_bounds_;        // 规划空间最小边界
    Eigen::Vector3d max_bounds_;        // 规划空间最大边界
    Eigen::Vector3i grid_size_;         // 栅格大小
    std::vector<Obstacle> obstacles_A;   // 障碍物列表
};
