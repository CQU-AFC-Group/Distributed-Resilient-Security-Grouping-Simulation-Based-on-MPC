#include "offboard_pkg/px4ctrl_utils.h"     // 相对路径

// px4ctrl
Px4ctrl_utils::Px4ctrl_utils(const ros::NodeHandle& nh, PX4CtrlFSM& fsm, const Parameter_t& param, const std::string uav_name)
    : nh_ns_("/" + uav_name), fsm_(fsm), param_(param), uav_name_(uav_name), nh_rc(nh)
{
      state_sub =
        nh_ns_.subscribe<mavros_msgs::State>("mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm_.state_data, _1));

     extended_state_sub =
        nh_ns_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm_.extended_state_data, _1));

     odom_sub =
        nh_ns_.subscribe<nav_msgs::Odometry>("mavros/local_position/odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm_.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

     cmd_sub =
        nh_ns_.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm_.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

     imu_sub =
        nh_ns_.subscribe<sensor_msgs::Imu>("mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm_.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    if (!param_.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh_rc.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm_.rc_data, _1));
    }

     bat_sub =
        nh_ns_.subscribe<sensor_msgs::BatteryState>("mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm_.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

     takeoff_land_sub =
        nh_rc.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm_.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());

    fsm_.ctrl_FCU_pub = nh_ns_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    fsm_.traj_start_trigger_pub = nh_ns_.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);

    fsm_.debug_pub = nh_ns_.advertise<quadrotor_msgs::Px4ctrlDebug>("debugPx4ctrl", 10); // debug

    fsm_.set_FCU_mode_srv = nh_ns_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    fsm_.arming_client_srv = nh_ns_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    fsm_.reboot_FCU_srv = nh_ns_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

}

void Px4ctrl_utils::PX4Ctrl_process()
{
     

    ros::Duration(5).sleep();

    if (param_.takeoff_land.no_RC)
    {
        ROS_WARN("[%s] PX4CTRL] Remote controller disabled, be careful!", uav_name_.c_str());

    }
    else
    {
        ROS_INFO("[%s] PX4CTRL] Waiting for RC", uav_name_.c_str());
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm_.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[%s] [PX4CTRL] RC received.", uav_name_.c_str());
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm_.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("[%s] Unable to connnect to PX4!!!", uav_name_.c_str());
    }
}
