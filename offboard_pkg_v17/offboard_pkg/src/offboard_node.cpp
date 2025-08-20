/**
 * @file offb_node.cpp
 * @brief Offboard control example node with circular trajectory, for MAVROS + PX4
 */

 #include <ros/ros.h>  // ROS 基础功能头文件
 #include <geometry_msgs/PoseStamped.h>  // 用于发布位置目标的消息类型
 #include <mavros_msgs/CommandBool.h>  // 用于发送解锁指令的服务类型
 #include <mavros_msgs/SetMode.h>  // 用于设置飞行模式的服务类型
 #include <mavros_msgs/State.h>  // 飞控当前状态的消息类型
 #include <math.h>  // 提供 sin 和 cos 函数用于生成轨迹
 
 mavros_msgs::State current_state;  // 全局变量，用于保存当前飞控状态
 
 // 回调函数：更新当前状态变量
 void state_cb(const mavros_msgs::State::ConstPtr& msg){
     current_state = *msg;  // 将接收到的状态赋值给全局变量
 }
 
 int main(int argc, char **argv)
 {
     ros::init(argc, argv, "offb_node");  // 初始化 ROS 节点，命名为 "offb_node"
     ros::NodeHandle nh;  // 创建 ROS 节点句柄
 
     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
         "mavros/state", 10, state_cb);  // 订阅 mavros/state 话题以获取飞控状态
 
     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
         "mavros/setpoint_position/local", 10);  // 发布器，向飞控发送本地位置目标点
 
     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
         "mavros/cmd/arming");  // 服务客户端，用于解锁飞控
 
     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
         "mavros/set_mode");  // 服务客户端，用于设置飞控模式
 
     ros::Rate rate(20.0);  // 控制频率为 20Hz，必须大于 2Hz 才能使用 OFFBOARD 模式
 
     while(ros::ok() && !current_state.connected){  // 等待与飞控建立连接
         ros::spinOnce();  // 处理回调函数
         rate.sleep();  // 按照设定频率睡眠
     }
 
     geometry_msgs::PoseStamped pose;  // 创建位置目标消息对象
     pose.pose.position.x = 0;  // 初始位置 X = 0
     pose.pose.position.y = 0;  // 初始位置 Y = 0
     pose.pose.position.z = 2;  // 起飞高度 Z = 2 米
 
     for(int i = 100; ros::ok() && i > 0; --i){  // 连续发送目标点确保进入 OFFBOARD 模式
         local_pos_pub.publish(pose);  // 发布当前位置目标
         ros::spinOnce();  // 处理回调函数
         rate.sleep();  // 睡眠以维持频率
     }
 
     mavros_msgs::SetMode offb_set_mode;  // 创建模式切换请求对象
     offb_set_mode.request.custom_mode = "OFFBOARD";  // 设置为 OFFBOARD 模式
 
     mavros_msgs::CommandBool arm_cmd;  // 创建解锁命令请求对象
     arm_cmd.request.value = true;  // 设置解锁命令为 true
 
     ros::Time last_request = ros::Time::now();  // 记录上次请求时间
 
     double radius = 3.0;  // 设置圆形轨迹半径为 3 米
     double omega = 0.2;  // 设置角速度为 0.2 rad/s
     ros::Time start_time = ros::Time::now();  // 记录轨迹开始时间
 
     while(ros::ok()){  // 主控制循环
         if( current_state.mode != "OFFBOARD" &&
             (ros::Time::now() - last_request > ros::Duration(5.0))){  // 如果未进入 OFFBOARD 且距离上次请求已超过 5 秒
             if( set_mode_client.call(offb_set_mode) &&
                 offb_set_mode.response.mode_sent){  // 尝试切换模式
                 ROS_INFO("Offboard enabled");  // 控制台输出成功信息
             }
             last_request = ros::Time::now();  // 更新请求时间
         } else {
             if( !current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0))){  // 如果尚未解锁且超过 5 秒
                 if( arming_client.call(arm_cmd) &&
                     arm_cmd.response.success){  // 尝试解锁
                     ROS_INFO("Vehicle armed");  // 控制台输出解锁信息
                 }
                 last_request = ros::Time::now();  // 更新请求时间
             }
         }
 
         ros::Duration elapsed = ros::Time::now() - start_time;  // 计算飞行时间
         double t = elapsed.toSec();  // 转换为秒数
 
         pose.pose.position.x = radius * cos(omega * t);  // X 坐标根据圆形轨迹计算
         pose.pose.position.y = radius * sin(omega * t);  // Y 坐标根据圆形轨迹计算
         pose.pose.position.z = 2.0;  // 高度保持为 2 米
 
         local_pos_pub.publish(pose);  // 发布当前轨迹点
 
         ros::spinOnce();  // 处理 ROS 回调函数
         rate.sleep();  // 控制循环频率
     }
 
     return 0;  // 程序正常结束
 }
 