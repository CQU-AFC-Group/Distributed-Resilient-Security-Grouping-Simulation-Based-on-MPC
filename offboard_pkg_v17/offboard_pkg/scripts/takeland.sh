source devel/setup.bash

rostopic pub -1 /takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
