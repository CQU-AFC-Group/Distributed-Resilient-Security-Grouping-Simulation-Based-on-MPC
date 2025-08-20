#include "offboard_pkg/topology_manager.h"
#include <sstream>
#include <iomanip>

TopologyManager::TopologyManager(ros::NodeHandle& nh) : nh_(nh), online_count_(0), offline_count_(0){
    // 从参数服务器加载拓扑配置
    loadTopologyFromParamServer();
    
    // 创建定时器，定期打印拓扑信息
    update_timer_ = nh_.createTimer(ros::Duration(2.0), 
                                   &TopologyManager::updateTimerCallback, this);
    
    ROS_INFO("[TopologyManager] Topology Manager initialized");
}

// 相关回调函数
void TopologyManager::updateTimerCallback(const ros::TimerEvent& event) {
    printTopologyInfo(); // 打印拓扑信息
    printOnlineStats(); // 打印获取并处理后的无人机在线状态信息
}

TopologyManager::~TopologyManager() {
    ROS_INFO("[TopologyManager] Topology Manager destroyed");
}

void TopologyManager::loadTopologyFromParamServer() {
    // 从ROS参数服务器加载拓扑信息
    std::vector<std::string> uav_names = {"uav0", "uav1", "uav2", "uav3", "uav4", 
                                          "uav5", "uav6", "uav7", "uav8", "uav9"};
    
    for (const auto& uav_name : uav_names) {
        std::string param_name = "/" + uav_name + "_node/neighbors";
        std::vector<std::string> neighbors;
        
        if (nh_.getParam(param_name, neighbors)) {
            UAVTopologyInfo uav_info;
            uav_info.uav_name = uav_name;
            uav_info.neighbors = neighbors;
            
            for (const auto& neighbor : neighbors) {
                uav_info.neighbor_states[neighbor] = NeighborInfo(neighbor, "UNKNOWN");
            }
            
            uav_topology_map_[uav_name] = uav_info;
            
            // 订阅该UAV的邻居状态上报
            std::string topic_name = "/" + uav_name + "/neighbor_states_report";
            ros::Subscriber sub = nh_.subscribe<std_msgs::String>(
                topic_name, 10, 
                boost::bind(&TopologyManager::handleNeighborStateReport, this, _1));
            neighbor_report_subs_.push_back(sub);
            
            ROS_INFO("[TopologyManager] Loaded UAV: %s with %lu neighbors", 
                     uav_name.c_str(), neighbors.size());
        } else {
            ROS_WARN("[TopologyManager] No neighbors found for %s", uav_name.c_str());
        }
    }
}

void TopologyManager::handleNeighborStateReport(const std_msgs::String::ConstPtr& msg) {
    std::string reporter_uav;
    std::map<std::string, std::string> neighbor_states;
    
    if (parseNeighborReport(msg->data, reporter_uav, neighbor_states)) {
        // 更新拓扑信息
        if (uav_topology_map_.find(reporter_uav) != uav_topology_map_.end()) {
            UAVTopologyInfo& uav_info = uav_topology_map_[reporter_uav];
            uav_info.last_report_time = ros::Time::now();
            
            for (const auto& state_pair : neighbor_states) {
                const std::string& neighbor_name = state_pair.first;
                const std::string& state = state_pair.second;
                
                if (uav_info.neighbor_states.find(neighbor_name) != uav_info.neighbor_states.end()) {
                    uav_info.neighbor_states[neighbor_name].state = state;
                    uav_info.neighbor_states[neighbor_name].last_update = ros::Time::now();
                }
            }
        }
    }
}

bool TopologyManager::parseNeighborReport(const std::string& msg_data, 
                                        std::string& reporter_uav, 
                                        std::map<std::string, std::string>& neighbor_states) {
    try {
        // 消息格式: "uav0:uav1=HOVERING,uav9=EXECUTING"
        size_t colon_pos = msg_data.find(':');
        if (colon_pos == std::string::npos) {
            return false;
        }
        
        reporter_uav = msg_data.substr(0, colon_pos);
        std::string states_str = msg_data.substr(colon_pos + 1);
        
        // 解析邻居状态信息
        std::stringstream ss(states_str);
        std::string pair;
        
        while (std::getline(ss, pair, ',')) {
            size_t eq_pos = pair.find('=');
            if (eq_pos != std::string::npos) {
                std::string neighbor = pair.substr(0, eq_pos);
                std::string state = pair.substr(eq_pos + 1);
                neighbor_states[neighbor] = state;
            }
        }
        
        return true;
    } catch (...) {
        ROS_ERROR("[TopologyManager] Failed to parse neighbor report: %s", msg_data.c_str());
        return false;
    }
}

void TopologyManager::printTopologyInfo() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "                    TOPOLOGY STATUS REPORT" << std::endl;
    std::cout << "                    " << ros::Time::now() << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    for (const auto& uav_pair : uav_topology_map_) {
        const UAVTopologyInfo& uav_info = uav_pair.second;
        
        std::cout << std::left << std::setw(8) << uav_info.uav_name << " | ";
        
        bool first = true;
        for (const auto& neighbor_pair : uav_info.neighbor_states) {
            if (!first) std::cout << ", ";
            
            const NeighborInfo& neighbor_info = neighbor_pair.second;
            std::cout << neighbor_info.neighbor_name << "=" << neighbor_info.state;
            first = false;
        }
        std::cout << std::endl;
    }
    
    std::cout << std::string(80, '-') << std::endl;
}

// 从邻居状态报告中收集所有UAV的状态信息
std::map<std::string, std::string> TopologyManager::collectAllUAVStates() {
    std::map<std::string, std::string> all_uav_states;
    
    // 遍历所有UAV的拓扑信息，收集被报告的邻居状态
    for (const auto& uav_pair : uav_topology_map_) {
        const UAVTopologyInfo& uav_info = uav_pair.second;
        
        // 从该UAV的邻居状态报告中提取状态信息
        for (const auto& neighbor_pair : uav_info.neighbor_states) {
            const std::string& neighbor_name = neighbor_pair.second.neighbor_name;
            const std::string& neighbor_state = neighbor_pair.second.state;
            
            // 如果这个邻居的状态不是UNKNOWN，就记录下来
            if (neighbor_state != "UNKNOWN") {
                // 如果已经有这个UAV的状态记录，检查时间戳，保留最新的
                if (all_uav_states.find(neighbor_name) == all_uav_states.end()) {
                    all_uav_states[neighbor_name] = neighbor_state;
                }
                else {
                    all_uav_states[neighbor_name] = neighbor_state;
                }
            }
        }
    }
    
    // 对于没有被任何邻居报告状态的UAV，标记为UNKNOWN
    std::vector<std::string> all_uav_names = {"uav0", "uav1", "uav2", "uav3", "uav4", 
                                              "uav5", "uav6", "uav7", "uav8", "uav9"};
    for (const auto& uav_name : all_uav_names) {
        if (all_uav_states.find(uav_name) == all_uav_states.end()) {
            all_uav_states[uav_name] = "UNKNOWN";
        }
    }
    return all_uav_states;
}

// 判断是否为在线状态
bool TopologyManager::isOnlineState(const std::string& state) {
    return (state == "INITIALIZING" || state == "PRE_PLANNING" || 
            state == "PLANNING" || state == "TAKING_OFF" || state == "HOVERING" || 
            state == "EXECUTING" || state == "FORMATION_CHANGING");
}

// 打印在线状态统计
void TopologyManager::printOnlineStats() {
    // 从邻居状态报告中收集所有UAV的状态
    std::map<std::string, std::string> all_uav_states = collectAllUAVStates();
    
    // 🔥 清空之前的统计信息
    online_uavs_.clear();
    offline_uavs_.clear();
    
    // 分类UAV并存储到成员变量
    for (const auto& uav_state : all_uav_states) {
        const std::string& uav_name = uav_state.first;
        const std::string& state = uav_state.second;
        
        if (isOnlineState(state)) {
            online_uavs_.push_back(uav_name);
        } else {
            offline_uavs_.push_back(uav_name);
        }
    }
    
    // 🔥 更新计数
    online_count_ = online_uavs_.size();
    offline_count_ = offline_uavs_.size();
    last_stats_update_ = ros::Time::now();
    
    // 打印在线UAV
    std::cout << "Online UAVs (" << online_count_ << "): ";
    if (online_uavs_.empty()) {
        std::cout << "None";
    } else {
        for (size_t i = 0; i < online_uavs_.size(); ++i) {
            std::cout << online_uavs_[i];
            if (i < online_uavs_.size() - 1) std::cout << ", ";
        }
    }
    std::cout << std::endl;
    
    // 打印离线UAV
    std::cout << "Offline UAVs (" << offline_count_ << "): ";
    if (offline_uavs_.empty()) {
        std::cout << "None";
    } else {
        for (size_t i = 0; i < offline_uavs_.size(); ++i) {
            std::cout << offline_uavs_[i];
            if (i < offline_uavs_.size() - 1) std::cout << ", ";
        }
    }
    std::cout << std::endl;
    
    std::cout << std::string(80, '=') << std::endl;
}
