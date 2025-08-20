// 在 uav_state_machine.h 中添加
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <geometry_msgs/TwistStamped.h>
#include "offboard_pkg/custom_trajectory_planner.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <nlopt.hpp>//求解器
#include <casadi/casadi.hpp>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include "MPCController.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "offboard_pkg/topology_manager.h" 


class TopologyManager;

class FormationCalculator {
public:
    // 计算圆形编队偏移量
    static std::map<std::string, Eigen::Vector3d> calculateCircleFormation(
        int num_uavs, double radius, const Eigen::Vector3d& center = Eigen::Vector3d::Zero());
    
    // 计算直线编队偏移量
    static std::map<std::string, Eigen::Vector3d> calculateLineFormation(
        int num_uavs, double spacing, const std::string& direction = "x", 
        const Eigen::Vector3d& center = Eigen::Vector3d::Zero(), double radius = 2.0);
    
    // 从参数服务器加载编队参数并计算偏移量
    static std::map<std::string, Eigen::Vector3d> calculateFormationFromParams(
        ros::NodeHandle& nh, const std::string& param_namespace = "formation");

    // 动态圆形编队计算（基于在线无人机列表）
    static std::map<std::string, Eigen::Vector3d> calculateDynamicCircleFormation(
        const std::vector<std::string>& online_uavs,
        double radius, 
        const Eigen::Vector3d& center);
    
    // 动态直线编队计算（基于在线无人机列表）
    static std::map<std::string, Eigen::Vector3d> calculateDynamicLineFormation(
        const std::vector<std::string>& online_uavs,
        double spacing, 
        const std::string& direction, 
        const Eigen::Vector3d& center, 
        double radius);
    
    // 从TopologyManager获取信息并计算动态编队
    static std::map<std::string, Eigen::Vector3d> calculateDynamicFormationFromTopology(
        const TopologyManager* topology_manager,
        const std::string& formation_type,
        ros::NodeHandle& nh,
        const std::string& param_namespace = "formation");
    
private:
    static std::string getUAVName(int index) { return "uav" + std::to_string(index); }
    static std::vector<std::string> sortUAVsByID(const std::vector<std::string>& uav_list);
};