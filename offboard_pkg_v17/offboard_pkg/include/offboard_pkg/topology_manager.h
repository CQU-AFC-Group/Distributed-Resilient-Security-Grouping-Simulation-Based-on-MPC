#ifndef TOPOLOGY_MANAGER_H
#define TOPOLOGY_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map>
#include <vector>
#include <string>

struct NeighborInfo {
    std::string neighbor_name;
    std::string state;
    ros::Time last_update;
    
    NeighborInfo() : state("UNKNOWN"), last_update(ros::Time::now()) {}
    NeighborInfo(const std::string& name, const std::string& s) 
        : neighbor_name(name), state(s), last_update(ros::Time::now()) {}
};

struct UAVTopologyInfo {
    std::string uav_name;
    std::vector<std::string> neighbors;
    std::map<std::string, NeighborInfo> neighbor_states;
    ros::Time last_report_time;
    
    UAVTopologyInfo() : last_report_time(ros::Time::now()) {}
};

class TopologyManager {
public:
    TopologyManager(ros::NodeHandle& nh);
    ~TopologyManager();
    
    // 从参数服务器加载拓扑结构
    void loadTopologyFromParamServer();
    
    // 处理无人机上报的邻居状态信息
    void handleNeighborStateReport(const std_msgs::String::ConstPtr& msg);
    
    // 打印拓扑信息
    void printTopologyInfo();

    // 存储在线/离线统计信息
    std::vector<std::string> online_uavs_;
    std::vector<std::string> offline_uavs_;
    int online_count_;
    int offline_count_;
    ros::Time last_stats_update_;

    // 为FormationManager提供的接口
    const std::vector<std::string>& getOnlineUAVs() const { return online_uavs_; }
    const std::vector<std::string>& getOfflineUAVs() const { return offline_uavs_; }
    int getOnlineCount() const { return online_count_; }
    int getOfflineCount() const { return offline_count_; }
    ros::Time getLastStatsUpdate() const { return last_stats_update_; }
    
    // 检查特定UAV是否在线
    bool isUAVOnline(const std::string& uav_name) const {
        return std::find(online_uavs_.begin(), online_uavs_.end(), uav_name) != online_uavs_.end();}

private:
    ros::NodeHandle nh_;
    
    // 存储所有无人机的拓扑信息
    std::map<std::string, UAVTopologyInfo> uav_topology_map_;
    
    // 订阅所有无人机的邻居状态上报
    std::vector<ros::Subscriber> neighbor_report_subs_;

    // 订阅UAV状态话题
    std::vector<ros::Subscriber> uav_state_subs_;
    
    // 解析上报消息
    bool parseNeighborReport(const std::string& msg_data, 
                           std::string& reporter_uav, 
                           std::map<std::string, std::string>& neighbor_states);

    //  从邻居状态报告中收集所有UAV的状态信息
    std::map<std::string, std::string> collectAllUAVStates();
    
    // 判断是否为在线状态
    bool isOnlineState(const std::string& state);
    
    // 打印在线状态统计
    void printOnlineStats();
    
    // 定时器
    ros::Timer update_timer_;
    void updateTimerCallback(const ros::TimerEvent& event);
};

#endif // TOPOLOGY_MANAGER_H
