#include "offboard_pkg/formation_manager.h"

// FormationCalculator 实现
std::map<std::string, Eigen::Vector3d> FormationCalculator::calculateCircleFormation(
    int num_uavs, double radius, const Eigen::Vector3d& center) {
    
    std::map<std::string, Eigen::Vector3d> offsets;
    
    for (int i = 0; i < num_uavs; ++i) {
        // 计算角度 (从0开始，逆时针分布)
        double angle = 2.0 * M_PI * i / num_uavs;
        // 计算偏移量
        double offset_x = center.x() + radius * cos(angle);
        double offset_y = center.y() + radius * sin(angle);
        double offset_z = center.z();
        std::string uav_name = getUAVName(i);
        offsets[uav_name] = Eigen::Vector3d(offset_x, offset_y, offset_z);
    }
    return offsets;
}

std::map<std::string, Eigen::Vector3d> FormationCalculator::calculateLineFormation(
    int num_uavs, double spacing, const std::string& direction, const Eigen::Vector3d& center, double radius) {
    std::map<std::string, Eigen::Vector3d> offsets;

    // 0号无人机保持圆形编队位置不变
    Eigen::Vector3d uav0_position;
    if (direction == "x") {
        uav0_position = Eigen::Vector3d(center.x(), center.y() + radius, center.z());
    } else { // direction == "y"
        uav0_position = Eigen::Vector3d(center.x() + radius, center.y(), center.z());
    }
    
    std::string uav0_name = getUAVName(0);
    offsets[uav0_name] = uav0_position;

    // 创建其他无人机的排列顺序
    std::vector<int> line_order;
    
    // 左侧：6, 7, 8, 9（原本圆形编队中0号右侧的无人机，按升序排列）
    for (int i = num_uavs/2 + 1; i < num_uavs; ++i) {
        line_order.push_back(i);
    }
    
    // 右侧：1, 2, 3, 4, 5（原本圆形编队中0号左侧的无人机，按升序排列）
    for (int i = 1; i <= num_uavs/2; ++i) {
        line_order.push_back(i);
    }
    
    // 计算其他无人机位置（以0号无人机为中心）
    int total_other_uavs = line_order.size(); // 9架其他无人机
    int left_count = num_uavs/2 - 1; // 左侧4架 (6,7,8,9)
    
    for (int pos = 0; pos < total_other_uavs; ++pos) {
        int uav_id = line_order[pos];
        double offset_x, offset_y;
        
        if (direction == "x") {
            // 以0号无人机的x坐标为基准
            if (pos < left_count) {
                // 左侧无人机
                offset_x = uav0_position.x() - (left_count - pos) * spacing;
            } else {
                // 右侧无人机
                offset_x = uav0_position.x() + (pos - left_count + 1) * spacing;
            }
            offset_y = uav0_position.y();
        } else { // direction == "y"
            // 以0号无人机的y坐标为基准
            if (pos < left_count) {
                // 左侧（上方）无人机
                offset_y = uav0_position.y() - (left_count - pos) * spacing;
            } else {
                // 右侧（下方）无人机
                offset_y = uav0_position.y() + (pos - left_count + 1) * spacing;
            }
            offset_x = uav0_position.x();
        }
        
        std::string uav_name = getUAVName(uav_id);
        offsets[uav_name] = Eigen::Vector3d(offset_x, offset_y, center.z());
    }
    
    return offsets;
}

std::map<std::string, Eigen::Vector3d> FormationCalculator::calculateFormationFromParams(
    ros::NodeHandle& nh, const std::string& param_namespace) {
    
    std::string formation_type;
    int num_uavs;
    
    // 读取基本配置
    nh.param(param_namespace + "/type", formation_type, std::string("circle"));
    nh.param(param_namespace + "/num_uavs", num_uavs, 10);
    
    std::map<std::string, Eigen::Vector3d> offsets;
    
    if (formation_type == "circle") {
        // 读取圆形编队参数
        double radius;
        std::vector<double> center_vec;
        
        nh.param(param_namespace + "/circle/radius", radius, 2.0);
        nh.param(param_namespace + "/circle/center", center_vec, std::vector<double>{0.0, 0.0, 0.0});
        
        Eigen::Vector3d center(center_vec[0], center_vec[1], center_vec[2]);
        offsets = calculateCircleFormation(num_uavs, radius, center);
        
        ROS_INFO("Generated circle formation: %d UAVs, radius=%.2f, center=(%.1f,%.1f,%.1f)", 
                    num_uavs, radius, center.x(), center.y(), center.z());
                    
    } else if (formation_type == "line") {
        // 读取直线编队参数
        double spacing, radius;
        std::string direction;
        std::vector<double> center_vec;
        
        nh.param(param_namespace + "/line/spacing", spacing, 1.5);
        nh.param(param_namespace + "/line/direction", direction, std::string("y"));
        nh.param(param_namespace + "/line/center", center_vec, std::vector<double>{0.0, 0.0, 0.0});
        nh.param(param_namespace + "/line/radius", radius, 2.0);  // 用于确定0号无人机位置
        
        Eigen::Vector3d center(center_vec[0], center_vec[1], center_vec[2]);
        offsets = calculateLineFormation(num_uavs, spacing, direction, center, radius);
        
        ROS_INFO("Generated line formation: %d UAVs, spacing=%.2f, direction=%s, center=(%.1f,%.1f,%.1f)", 
                    num_uavs, spacing, direction.c_str(), center.x(), center.y(), center.z());
                    
    } 
    // 打印编队信息
    ROS_INFO("Formation offsets:");
    for (const auto& pair : offsets) {
        const auto& name = pair.first;
        const auto& offset = pair.second;
        ROS_INFO("  %s: (%.3f, %.3f, %.3f)", name.c_str(), 
                    offset.x(), offset.y(), offset.z());
    }
    
    return offsets;
}

// 辅助函数：按UAV ID排序
std::vector<std::string> FormationCalculator::sortUAVsByID(const std::vector<std::string>& uav_list) {
    std::vector<std::string> sorted_uavs = uav_list;
    std::sort(sorted_uavs.begin(), sorted_uavs.end(), 
              [](const std::string& a, const std::string& b) {
                  // 提取数字部分进行比较 (uav0, uav1, uav2, ...)
                  int num_a = std::stoi(a.substr(3)); // 去掉"uav"前缀
                  int num_b = std::stoi(b.substr(3));
                  return num_a < num_b;
              });
    
    return sorted_uavs;
}

// 动态圆形编队计算
std::map<std::string, Eigen::Vector3d> FormationCalculator::calculateDynamicCircleFormation(
    const std::vector<std::string>& online_uavs,
    double radius, 
    const Eigen::Vector3d& center) {
    
    std::map<std::string, Eigen::Vector3d> offsets;
    
    if (online_uavs.empty()) {
        ROS_WARN("[FormationCalculator] No online UAVs for dynamic circle formation");
        return offsets;
    }
    
    // 按ID排序在线无人机
    std::vector<std::string> sorted_uavs = sortUAVsByID(online_uavs);
    int num_online_uavs = sorted_uavs.size();
    
    ROS_INFO("[FormationCalculator] Calculating dynamic circle formation for %d online UAVs", 
             num_online_uavs);
    
    // 为每个在线无人机计算圆形编队位置
    for (int i = 0; i < num_online_uavs; ++i) {
        // 计算角度 (从0开始，逆时针分布)
        double angle = 2.0 * M_PI * i / num_online_uavs;
        
        // 计算偏移量
        double offset_x = center.x() + radius * cos(angle);
        double offset_y = center.y() + radius * sin(angle);
        double offset_z = center.z();
        
        const std::string& uav_name = sorted_uavs[i];
        offsets[uav_name] = Eigen::Vector3d(offset_x, offset_y, offset_z);
        
        ROS_INFO("[FormationCalculator] %s -> position %d: angle=%.2f°, offset=(%.3f, %.3f, %.3f)", 
                 uav_name.c_str(), i, angle * 180.0 / M_PI, 
                 offset_x, offset_y, offset_z);
    }
    
    return offsets;
}

// 动态直线编队计算
std::map<std::string, Eigen::Vector3d> FormationCalculator::calculateDynamicLineFormation(
    const std::vector<std::string>& online_uavs,
    double spacing, 
    const std::string& direction, 
    const Eigen::Vector3d& center, 
    double radius) {

    std::map<std::string, Eigen::Vector3d> offsets;

    if (online_uavs.empty()) {
        ROS_WARN("[FormationCalculator] No online UAVs for dynamic line formation");
        return offsets;
    }
    
    // 按ID排序在线无人机
    std::vector<std::string> sorted_uavs = sortUAVsByID(online_uavs);
    int num_online_uavs = sorted_uavs.size();
    
    // 检查uav0是否在线
    bool uav0_online = std::find(sorted_uavs.begin(), sorted_uavs.end(), "uav0") != sorted_uavs.end();
    
    // 0号无人机位置
    Eigen::Vector3d uav0_position;
    if (direction == "x") {
        uav0_position = Eigen::Vector3d(center.x(), center.y() + radius, center.z());
    } else { // direction == "y"
        uav0_position = Eigen::Vector3d(center.x() + radius, center.y(), center.z());
    }
    
    // 如果uav0在线，设置其位置
    if (uav0_online) {
        offsets["uav0"] = uav0_position;
    }
    
    // 提取其他在线无人机并按固定版本逻辑分组
    std::vector<std::string> other_online_uavs;
    for (const auto& uav : sorted_uavs) {
        if (uav != "uav0") {
            other_online_uavs.push_back(uav);
        }
    }
    
    if (other_online_uavs.empty()) {
        return offsets; // 只有uav0在线或没有其他无人机
    }
    
    // 按照固定版本的逻辑分组：基于实际的UAV ID
    std::vector<std::string> left_side_uavs;   // ID较大的无人机
    std::vector<std::string> right_side_uavs;  // ID较小的无人机
    
    // 假设总无人机数量为10，分界点为5
    int total_assumed_uavs = 10;
    int mid_point = total_assumed_uavs / 2; // 5
    
    for (const auto& uav : other_online_uavs) {
        int uav_id = std::stoi(uav.substr(3)); // 提取ID
        
        if (uav_id > mid_point) {
            // ID > 5: 左侧 (6,7,8,9,...)
            left_side_uavs.push_back(uav);
        } else {
            // ID <= 5: 右侧 (1,2,3,4,5)
            right_side_uavs.push_back(uav);
        }
    }
    
    // 各组内部按升序排列
    std::sort(left_side_uavs.begin(), left_side_uavs.end(), 
                [](const std::string& a, const std::string& b) {
                    return std::stoi(a.substr(3)) < std::stoi(b.substr(3));
                });
    
    std::sort(right_side_uavs.begin(), right_side_uavs.end(), 
                [](const std::string& a, const std::string& b) {
                    return std::stoi(a.substr(3)) < std::stoi(b.substr(3));
                });
    
    // 创建排列顺序：左侧 + 右侧
    std::vector<std::string> line_order;
    line_order.insert(line_order.end(), left_side_uavs.begin(), left_side_uavs.end());
    line_order.insert(line_order.end(), right_side_uavs.begin(), right_side_uavs.end());
    
    // 计算位置
    int total_other_uavs = line_order.size();
    int left_count = left_side_uavs.size();
    
    // 选择基准位置：如果uav0在线用uav0位置，否则用中心
    Eigen::Vector3d reference_position = uav0_online ? uav0_position : center;
    
    for (int pos = 0; pos < total_other_uavs; ++pos) {
        const std::string& uav_name = line_order[pos];
        double offset_x, offset_y;
        
        if (direction == "x") {
            if (pos < left_count) {
                // 左侧无人机
                offset_x = reference_position.x() - (left_count - pos) * spacing;
            } else {
                // 右侧无人机
                offset_x = reference_position.x() + (pos - left_count + 1) * spacing;
            }
            offset_y = reference_position.y();
        } else { // direction == "y"
            if (pos < left_count) {
                // 左侧（上方）无人机
                offset_y = reference_position.y() - (left_count - pos) * spacing;
            } else {
                // 右侧（下方）无人机
                offset_y = reference_position.y() + (pos - left_count + 1) * spacing;
            }
            offset_x = reference_position.x();
        }
        
        offsets[uav_name] = Eigen::Vector3d(offset_x, offset_y, center.z());
        
        ROS_INFO("[FormationCalculator] %s -> %s side, pos %d: (%.3f, %.3f, %.3f)", 
                    uav_name.c_str(), 
                    (pos < left_count) ? "left" : "right",
                    pos, offset_x, offset_y, center.z());
    }
    return offsets;
}

// 从TopologyManager获取信息并计算动态编队
std::map<std::string, Eigen::Vector3d> FormationCalculator::calculateDynamicFormationFromTopology(
    const TopologyManager* topology_manager,
    const std::string& formation_type,
    ros::NodeHandle& nh,
    const std::string& param_namespace) {
    
    std::map<std::string, Eigen::Vector3d> offsets;
    
    if (!topology_manager) {
        ROS_ERROR("[FormationCalculator] TopologyManager is null");
        return offsets;
    }
    
    //  从TopologyManager获取在线无人机列表
    std::vector<std::string> online_uavs = topology_manager->getOnlineUAVs();
    int online_count = topology_manager->getOnlineCount();
    
    if (online_count == 0) {
        ROS_WARN("[FormationCalculator] No online UAVs reported by TopologyManager");
        return offsets;
    }
    
    ROS_INFO("[FormationCalculator] Received %d online UAVs from TopologyManager", online_count);
    
    if (formation_type == "circle") {
        // 读取圆形编队参数
        double radius;
        std::vector<double> center_vec;
        
        nh.param(param_namespace + "/circle/radius", radius, 2.0);
        nh.param(param_namespace + "/circle/center", center_vec, std::vector<double>{0.0, 0.0, 0.0});
        
        Eigen::Vector3d center(center_vec[0], center_vec[1], center_vec[2]);
        
        //  调用动态圆形编队计算
        offsets = calculateDynamicCircleFormation(online_uavs, radius, center);
        
        ROS_INFO("[FormationCalculator] Generated dynamic circle formation: %d UAVs, radius=%.2f", 
                 online_count, radius);
                 
    } else if (formation_type == "line") {
        // 读取直线编队参数
        double spacing, radius;
        std::string direction;
        std::vector<double> center_vec;
        
        nh.param(param_namespace + "/line/spacing", spacing, 1.5);
        nh.param(param_namespace + "/line/direction", direction, std::string("y"));
        nh.param(param_namespace + "/line/center", center_vec, std::vector<double>{0.0, 0.0, 0.0});
        nh.param(param_namespace + "/line/radius", radius, 2.0);
        
        Eigen::Vector3d center(center_vec[0], center_vec[1], center_vec[2]);
        
        //  调用动态直线编队计算
        offsets = calculateDynamicLineFormation(online_uavs, spacing, direction, center, radius);
        
        ROS_INFO("[FormationCalculator] Generated dynamic line formation: %d UAVs, spacing=%.2f", 
                 online_count, spacing);
    } else {
        ROS_ERROR("[FormationCalculator] Unsupported formation type: %s", formation_type.c_str());
    }
    
    // 打印编队信息
    if (!offsets.empty()) {
        ROS_INFO("[FormationCalculator] Dynamic formation offsets:");
        for (const auto& pair : offsets) {
            const auto& name = pair.first;
            const auto& offset = pair.second;
            ROS_INFO("  %s: (%.3f, %.3f, %.3f)", name.c_str(), 
                     offset.x(), offset.y(), offset.z());
        }
    }
    
    return offsets;
}