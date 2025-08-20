// astar_planner.cpp
#include "offboard_pkg/astar_planner.h"
#include <cmath>
#include <algorithm>
#include <iostream>

AStarPlanner::AStarPlanner(double resolution)
    : resolution_(resolution) {
    // 默认规划空间
    min_bounds_ = Eigen::Vector3d(-10, -10, 0);
    max_bounds_ = Eigen::Vector3d(10, 10, 10);
    
    // 计算栅格大小
    grid_size_ = ((max_bounds_ - min_bounds_) / resolution_).cast<int>() + Eigen::Vector3i::Ones();
}

AStarPlanner::~AStarPlanner() {}

void AStarPlanner::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_A = obstacles;
}

void AStarPlanner::setPlanningBounds(const Eigen::Vector3d& min_bounds, const Eigen::Vector3d& max_bounds) {
    min_bounds_ = min_bounds;
    max_bounds_ = max_bounds;
    
    // 更新栅格大小
    grid_size_ = ((max_bounds_ - min_bounds_) / resolution_).cast<int>() + Eigen::Vector3i::Ones();
}

bool AStarPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& path) {
    // 将起点和终点转换为栅格索引
    Eigen::Vector3i start_index = posToIndex(start);
    Eigen::Vector3i goal_index = posToIndex(goal);
     // 保存原始Z值，用于后续生成路径点
    double fixed_z = start.z();
    
    // 检查起点和终点是否有效
    if (!isIndexValid(start_index) || !isIndexValid(goal_index)) {
        std::cerr << "Start or goal position is outside planning bounds!" << std::endl;
        return false;
    }
    
    // 检查起点和终点是否无碰撞
    if (!isCollisionFree(start) || !isCollisionFree(goal)) {
        std::cerr << "Start or goal position is in collision with obstacles!" << std::endl;
        return false;
    }
    
    // 创建开始节点
    Node* start_node = new Node(start_index, 0.0, calculateHeuristic(start_index, goal_index));
    
    // 优先队列（open set）
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> open_set;
    open_set.push(start_node);
    
    // 已访问节点集合（closed set）
    std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> closed_set;
    
    // 记录节点映射，用于检查节点是否在open set中
    std::unordered_map<Eigen::Vector3i, Node*, Vector3iHash, Vector3iEqual> node_map;
    node_map[start_index] = start_node;
    
    bool found_path = false;
    Node* goal_node = nullptr;
    
    // A*主循环
    while (!open_set.empty()) {
        // 获取f值最小的节点
        Node* current = open_set.top();
        open_set.pop();
        
        // 如果到达目标
        if (current->index == goal_index) {
            found_path = true;
            goal_node = current;
            break;
        }
        
        // 将当前节点加入closed set
        closed_set.insert(current->index);
        
        // 获取相邻节点
        std::vector<Eigen::Vector3i> neighbors = getNeighbors(current->index);
        
        for (const auto& neighbor_index : neighbors) {
            // 如果邻居已在closed set中，跳过
            if (closed_set.find(neighbor_index) != closed_set.end()) {
                continue;
            }
            
            // 计算到邻居的代价
            Eigen::Vector3d neighbor_pos = indexToPos(neighbor_index);
            // 二维处理
            neighbor_pos.z() = fixed_z;  // 保持Z值不变
            
            // 检查邻居是否无碰撞
            if (!isCollisionFree(neighbor_pos)) {
                continue;
            }
            
            // 计算从起点经过当前节点到邻居的代价
            // 三维
            // double g_cost = current->g_cost + (indexToPos(current->index) - neighbor_pos).norm();
            // 二维
            double g_cost = current->g_cost + std::sqrt(std::pow(indexToPos(current->index).x() - neighbor_pos.x(), 2) + std::pow(indexToPos(current->index).y() - neighbor_pos.y(), 2));
            // 检查邻居是否已在open set中
            auto it = node_map.find(neighbor_index);
            if (it == node_map.end()) {
                // 创建新节点
                double h_cost = calculateHeuristic(neighbor_index, goal_index);
                Node* neighbor_node = new Node(neighbor_index, g_cost, h_cost, current);
                open_set.push(neighbor_node);
                node_map[neighbor_index] = neighbor_node;
            } else {
                // 更新现有节点
                Node* neighbor_node = it->second;
                if (g_cost < neighbor_node->g_cost) {
                    neighbor_node->g_cost = g_cost;
                    neighbor_node->f_cost = g_cost + neighbor_node->h_cost;
                    neighbor_node->parent = current;
                }
            }
        }
    }
    
    // 如果找到路径
    if (found_path && goal_node) {
        // 重建路径
        path = reconstructPath(goal_node, fixed_z);
        
        // 简化路径
        path = simplifyPath(path);
        
        // 释放内存
        for (auto& pair : node_map) {
            delete pair.second;
        }
        
        return true;
    }
    
    // 释放内存
    for (auto& pair : node_map) {
        delete pair.second;
    }
    
    std::cerr << "No path found from start to goal!" << std::endl;
    return false;
}

Eigen::Vector3i AStarPlanner::posToIndex(const Eigen::Vector3d& pos) const {
    return ((pos - min_bounds_) / resolution_).cast<int>();
}

Eigen::Vector3d AStarPlanner::indexToPos(const Eigen::Vector3i& index) const {
    return min_bounds_ + index.cast<double>() * resolution_ + Eigen::Vector3d(resolution_ / 2, resolution_ / 2, resolution_ / 2);
}

bool AStarPlanner::isIndexValid(const Eigen::Vector3i& index) const {
    return index.x() >= 0 && index.x() < grid_size_.x() &&
           index.y() >= 0 && index.y() < grid_size_.y() &&
           index.z() >= 0 && index.z() < grid_size_.z();
}

bool AStarPlanner::isCollisionFree(const Eigen::Vector3d& pos) const {
    // 三维
    // for (const auto& obstacle : obstacles_A) {
    //     double distance = (pos - obstacle.position).norm() - obstacle.radius;
    //     if (distance <= 0.5) {  // 安全裕度
    //         return false;
    //     }
    // }
    // return true;
    // 二维
     for (const auto& obstacle : obstacles_A) 
     {
        double distance_xy = std::sqrt(
            std::pow(pos.x() - obstacle.position.x(), 2) + 
            std::pow(pos.y() - obstacle.position.y(), 2));
        
        if (distance_xy <= obstacle.radius + 0.4) {  // 安全裕度
            return false;
        }
    }
    return true;
}

double AStarPlanner::calculateHeuristic(const Eigen::Vector3i& index, const Eigen::Vector3i& goal_index) const {
    // 使用欧氏距离作为启发式函数
    // 三维
    // return (indexToPos(index) - indexToPos(goal_index)).norm();
    //二维
     // 只考虑XY平面的欧氏距离
     Eigen::Vector3d pos = indexToPos(index);
     Eigen::Vector3d goal_pos = indexToPos(goal_index);
     return std::sqrt(std::pow(pos.x() - goal_pos.x(), 2) + std::pow(pos.y() - goal_pos.y(), 2));
}

std::vector<Eigen::Vector3i> AStarPlanner::getNeighbors(const Eigen::Vector3i& index) const {
    std::vector<Eigen::Vector3i> neighbors;
    // 三维
    // // 6-连通邻居（上下左右前后）
    // const std::vector<Eigen::Vector3i> directions = {
    //     Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
    //     Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
    //     Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)
    // };
    
    // for (const auto& dir : directions) {
    //     Eigen::Vector3i neighbor = index + dir;
    //     if (isIndexValid(neighbor)) {
    //         neighbors.push_back(neighbor);
    //     }
    // }
    
    // // 对角线方向的邻居
    // const std::vector<Eigen::Vector3i> diag_directions = {
    //     Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, -1, 0),
    //     Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, -1, 0),
    //     Eigen::Vector3i(1, 0, 1), Eigen::Vector3i(1, 0, -1),
    //     Eigen::Vector3i(-1, 0, 1), Eigen::Vector3i(-1, 0, -1),
    //     Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(0, 1, -1),
    //     Eigen::Vector3i(0, -1, 1), Eigen::Vector3i(0, -1, -1)
    // };
    
    // for (const auto& dir : diag_directions) {
    //     Eigen::Vector3i neighbor = index + dir;
    //     if (isIndexValid(neighbor)) {
    //         // 检查对角线移动时的中间点
    //         bool safe_diagonal = true;
            
    //         // 分解对角线移动为基本移动
    //         for (int i = 0; i < 3; i++) {
    //             if (dir[i] != 0) {
    //                 Eigen::Vector3i intermediate = index;
    //                 intermediate[i] += dir[i];
                    
    //                 if (!isIndexValid(intermediate) || 
    //                     !isCollisionFree(indexToPos(intermediate))) {
    //                     safe_diagonal = false;
    //                     break;
    //                 }
    //             }
    //         }
            
    //         if (safe_diagonal) {
    //             neighbors.push_back(neighbor);
    //         }
    //     }
    // }
    
    // return neighbors;
    // 4-连通邻居（左右前后）- 只在XY平面移动
    const std::vector<Eigen::Vector3i> directions = {
        Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0)
    };
    
    for (const auto& dir : directions) {
        Eigen::Vector3i neighbor = index + dir;
        // 保持Z坐标不变
        neighbor.z() = index.z();
        
        if (isIndexValid(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    // 对角线方向的邻居 - 只在XY平面移动
    const std::vector<Eigen::Vector3i> diag_directions = {
        Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, -1, 0),
        Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, -1, 0)
    };
    
    for (const auto& dir : diag_directions) {
        Eigen::Vector3i neighbor = index + dir;
        // 保持Z坐标不变
        neighbor.z() = index.z();
        
        if (isIndexValid(neighbor)) {
            // 检查对角线移动时的中间点
            bool safe_diagonal = true;
            
            // 分解对角线移动为基本移动
            for (int i = 0; i < 2; i++) {  // 只检查XY两个方向
                if (dir[i] != 0) {
                    Eigen::Vector3i intermediate = index;
                    intermediate[i] += dir[i];
                    
                    if (!isIndexValid(intermediate) || 
                        !isCollisionFree(indexToPos(intermediate))) {
                        safe_diagonal = false;
                        break;
                    }
                }
            }
            
            if (safe_diagonal) {
                neighbors.push_back(neighbor);
            }
        }
    }
    
    return neighbors;
}

// 三维
// std::vector<Eigen::Vector3d> AStarPlanner::reconstructPath(Node* goal_node) const {
//     std::vector<Eigen::Vector3d> path;
    
//     // 从目标节点回溯到起始节点
//     for (Node* node = goal_node; node != nullptr; node = node->parent) {
//         path.push_back(indexToPos(node->index));
//     }
    
//     // 反转路径，使其从起点到终点
//     std::reverse(path.begin(), path.end());
    
//     return path;
// }
// 二维
std::vector<Eigen::Vector3d> AStarPlanner::reconstructPath(Node* goal_node, double fixed_z) const {
    std::vector<Eigen::Vector3d> path;
    
    // 从目标节点回溯到起始节点
    for (Node* node = goal_node; node != nullptr; node = node->parent) {
        Eigen::Vector3d pos = indexToPos(node->index);
        pos.z() = fixed_z;  // 保持Z高度不变
        path.push_back(pos);
    }
    
    // 反转路径，使其从起点到终点
    std::reverse(path.begin(), path.end());
    
    return path;
}

std::vector<Eigen::Vector3d> AStarPlanner::simplifyPath(const std::vector<Eigen::Vector3d>& path) const {
    // 三维
    // if (path.size() <= 2) {
    //     return path;
    // }
    
    // std::vector<Eigen::Vector3d> simplified_path;
    // simplified_path.push_back(path.front());
    
    // for (size_t i = 1; i < path.size() - 1; ++i) {
    //     Eigen::Vector3d prev = simplified_path.back();
    //     Eigen::Vector3d curr = path[i];
    //     Eigen::Vector3d next = path[i + 1];
        
    //     // 检查跳过当前点是否安全
    //     bool is_safe = true;
        
    //     // 检查prev到next的直线是否无碰撞
    //     double step_size = resolution_ * 0.2;  // 采样步长
    //     double distance = (next - prev).norm();
    //     int steps = std::ceil(distance / step_size);
        
    //     for (int j = 1; j < steps; ++j) {
    //         Eigen::Vector3d check_point = prev + (next - prev) * (j * step_size / distance);
    //         if (!isCollisionFree(check_point)) {
    //             is_safe = false;
    //             break;
    //         }
    //     }
        
    //     // 如果不安全，保留当前点
    //     if (!is_safe) {
    //         simplified_path.push_back(curr);
    //     }
    // }
    
    // simplified_path.push_back(path.back());
    // return simplified_path;
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<Eigen::Vector3d> simplified_path;
    simplified_path.push_back(path.front());
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector3d prev = simplified_path.back();
        Eigen::Vector3d curr = path[i];
        Eigen::Vector3d next = path[i + 1];
        
        // 检查跳过当前点是否安全 - 只考虑XY平面
        bool is_safe = true;
        
        // 检查prev到next的直线是否无碰撞
        double step_size = resolution_ * 0.2;  // 采样步长
        double distance = std::sqrt(
            std::pow(next.x() - prev.x(), 2) + 
            std::pow(next.y() - prev.y(), 2));
            
        int steps = std::ceil(distance / step_size);
        
        for (int j = 1; j < steps; ++j) {
            // 在XY平面上进行线性插值
            Eigen::Vector3d check_point;
            check_point.x() = prev.x() + (next.x() - prev.x()) * (j * step_size / distance);
            check_point.y() = prev.y() + (next.y() - prev.y()) * (j * step_size / distance);
            check_point.z() = prev.z();  // 保持Z高度不变
            
            if (!isCollisionFree(check_point)) {
                is_safe = false;
                break;
            }
        }
        
        // 如果不安全，保留当前点
        if (!is_safe) {
            simplified_path.push_back(curr);
        }
    }
    
    simplified_path.push_back(path.back());
    return simplified_path;
}

// A*加编队约束
// 在文件末尾添加

void AStarPlanner::setFormationOffsets(const std::map<std::string, Eigen::Vector3d>& offsets) {
    formation_offsets_ = offsets;
}

double AStarPlanner::evaluateFormationConstraint(const Eigen::Vector3d& pos) const {
    // 假设pos是领导者的位置
    double constraint_violation = 0.0;
    
    for (const auto& offset_pair : formation_offsets_) {
        // 计算跟随者的预期位置
        Eigen::Vector3d follower_pos = pos + offset_pair.second;
        
        // 检查该位置是否无碰撞
        if (!isCollisionFree(follower_pos)) {
            // 如果有碰撞，增加违反约束的程度
            for (const auto& obstacle : obstacles_A) {
                double distance_xy = std::sqrt(
                    std::pow(follower_pos.x() - obstacle.position.x(), 2) + 
                    std::pow(follower_pos.y() - obstacle.position.y(), 2));
                
                double violation = std::max(0.0, obstacle.radius + 0.3 - distance_xy);
                constraint_violation += violation * 10.0;  // 乘以权重
            }
        }
    }
    
    return constraint_violation;
}

bool AStarPlanner::planWithFormationConstraints(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, 
                                               std::vector<Eigen::Vector3d>& path) {
    // 将起点和终点转换为栅格索引
    Eigen::Vector3i start_index = posToIndex(start);
    Eigen::Vector3i goal_index = posToIndex(goal);
    double fixed_z = start.z();
    
    // 检查起点和终点是否有效
    if (!isIndexValid(start_index) || !isIndexValid(goal_index)) {
        std::cerr << "Start or goal position is outside planning bounds!" << std::endl;
        return false;
    }
    
    // 检查起点和终点是否无碰撞（包括编队约束）
    if (!isCollisionFree(start) || !isCollisionFree(goal) || 
        evaluateFormationConstraint(start) > 0 || evaluateFormationConstraint(goal) > 0) {
        std::cerr << "Start or goal position violates formation constraints!" << std::endl;
        return false;
    }
    
    // 创建开始节点
    Node* start_node = new Node(start_index, 0.0, calculateHeuristic(start_index, goal_index));
    
    // 优先队列（open set）
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> open_set;
    open_set.push(start_node);
    
    // 已访问节点集合（closed set）
    std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> closed_set;
    
    // 记录节点映射
    std::unordered_map<Eigen::Vector3i, Node*, Vector3iHash, Vector3iEqual> node_map;
    node_map[start_index] = start_node;
    
    bool found_path = false;
    Node* goal_node = nullptr;
    
    // A*主循环
    while (!open_set.empty()) {
        // 获取f值最小的节点
        Node* current = open_set.top();
        open_set.pop();
        
        // 如果到达目标
        if (current->index == goal_index) {
            found_path = true;
            goal_node = current;
            break;
        }
        
        // 将当前节点加入closed set
        closed_set.insert(current->index);
        
        // 获取相邻节点
        std::vector<Eigen::Vector3i> neighbors = getNeighbors(current->index);
        
        for (const auto& neighbor_index : neighbors) {
            // 如果邻居已在closed set中，跳过
            if (closed_set.find(neighbor_index) != closed_set.end()) {
                continue;
            }
            
            // 计算到邻居的代价
            Eigen::Vector3d neighbor_pos = indexToPos(neighbor_index);
            neighbor_pos.z() = fixed_z;  // 保持Z值不变
            
            // 检查邻居是否无碰撞
            if (!isCollisionFree(neighbor_pos)) {
                continue;
            }
            
            // 评估编队约束
            double formation_constraint = evaluateFormationConstraint(neighbor_pos);
            
            // 如果编队约束违反太严重，跳过该节点
            if (formation_constraint > 3.0) {
                continue;
            }
            
            // 计算从起点经过当前节点到邻居的代价
            double g_cost = current->g_cost + 
                           std::sqrt(std::pow(indexToPos(current->index).x() - neighbor_pos.x(), 2) + 
                                    std::pow(indexToPos(current->index).y() - neighbor_pos.y(), 2));
            
            // 将编队约束纳入总代价
            double formation_weight = 0.2;  // 编队约束权重
            g_cost += formation_weight * formation_constraint;
            
            // 检查邻居是否已在open set中
            auto it = node_map.find(neighbor_index);
            if (it == node_map.end()) {
                // 创建新节点
                double h_cost = calculateHeuristic(neighbor_index, goal_index);
                Node* neighbor_node = new Node(neighbor_index, g_cost, h_cost, current);
                open_set.push(neighbor_node);
                node_map[neighbor_index] = neighbor_node;
            } else {
                // 更新现有节点
                Node* neighbor_node = it->second;
                if (g_cost < neighbor_node->g_cost) {
                    neighbor_node->g_cost = g_cost;
                    neighbor_node->f_cost = g_cost + neighbor_node->h_cost;
                    neighbor_node->parent = current;
                }
            }
        }
    }
    
    // 如果找到路径
    if (found_path && goal_node) {
        // 重建路径
        path = reconstructPath(goal_node, fixed_z);
        
        // 简化路径
        path = simplifyPath(path);
        
        // 优化路径以满足编队约束
        path = optimizePathForFormation(path);
        
        // 释放内存
        for (auto& pair : node_map) {
            delete pair.second;
        }
        
        return true;
    }
    
    // 释放内存
    for (auto& pair : node_map) {
        delete pair.second;
    }
    
    std::cerr << "No path found that satisfies formation constraints!" << std::endl;
    return false;
}

std::vector<Eigen::Vector3d> AStarPlanner::optimizePathForFormation(
    const std::vector<Eigen::Vector3d>& path) const {
    
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<Eigen::Vector3d> optimized_path = path;
    
    // 迭代优化
    for (int iter = 0; iter < 5; ++iter) {
        // 平滑处理
        for (size_t i = 1; i < optimized_path.size() - 1; ++i) {
            // 计算平滑后的位置
            Eigen::Vector3d smooth_pos = 0.25 * optimized_path[i-1] + 
                                        0.5 * optimized_path[i] + 
                                        0.25 * optimized_path[i+1];
            
            // 检查平滑后的位置是否满足约束
            bool is_valid = isCollisionFree(smooth_pos);
            
            // 检查编队约束
            if (evaluateFormationConstraint(smooth_pos) > 0) {
                is_valid = false;
            }
            
            // 如果有效，更新位置
            if (is_valid) {
                optimized_path[i] = smooth_pos;
            }
        }
    }
    
    return optimized_path;
}

//