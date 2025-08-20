# 设置动态障碍物状态
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'dynamic_cylinder'
  pose:
    position:
      x: 8.0
      y: -2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  twist:
    linear:
      x: -0.05
      y: 0.00
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0" &

# 启动分布式控制
rostopic pub /start_distributed_control std_msgs/Bool "data: true" --once

echo "仿真已启动！"