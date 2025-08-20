# Simulation Project Usage Instructions

## 1. Initial Startup Instructions

### 1. Dependencies

Currently, the project code relies on the following packages:  
- `offboard_pkg`
- `quadrotor_msgs`
- `sim_rc`
- `uav_utils`

Some `.cpp` and `.h` files from `px4ctrl` have been merged into `offboard_pkg`. In `main.cpp`, two startup modes are defined:

#### **Mode 1: CONTROL_MODE = 1**

1. Launch:
    ```bash
    roslaunch px4 multi_mavros_ten_circle.launch
    roslaunch offboard_pkg multi_offboard_circle.launch
    ```

#### **Mode 2: CONTROL_MODE = 2**

1. Open Terminal 1:
    ```bash
    roscd sim_rc/scripts/
    chmod +x sim_rc_node.py
    cd ../..
    source devel/setup.bash
    ```
2. Open Terminal 2:
    ```bash
    roscd offboard_pkg/scripts/
    chmod +x takeoff.sh
    chmod +x takeland.sh
    ```
3. Open Terminal 3:
    ```bash
    roslaunch offboard_pkg multi_offboard_circle.launch
    ```
    - Terminal 3 displays: `PX4CTRL Waiting for RC`
4. In Terminal 1:
    ```bash
    rosrun sim_rc sim_rc_node.py
    ```
    - Terminal 3 displays: `[PX4CTRL] RC received`
5. Open Terminal 4:
    ```bash
    roslaunch px4 multi_mavros_ten_circle.launch
    ```
    - Wait for Terminal 3 to display the drone battery status
6. In Terminal 1:
    ```
    5 2000
    6 2000
    ```
    - Terminal 3 indicates ready for takeoff
7. In Terminal 2:
    ```bash
    ./takeoff.sh
    ```
    - Terminal 3 shows drones entering `AUTO_TAKEOFF` mode
8. In Terminal 1:
    ```
    7 2000
    ```
    - Terminal 3 shows drones entering `CMD_CTRL` mode
9. In Terminal 1:
    ```
    7 1000
    ```
    - Drones exit `CMD_CTRL` mode
10. In Terminal 2:
    ```bash
    ./takeland.sh
    ```
    - Drones switch from `AUTO-HOVER` to `AUTO-LAND` mode

---

## 2. Important Parameter Modification Instructions (Workspace Example: `wjx_ws`)

### 1. Initial Position, Target Position, Arrival Time, Threshold, Takeoff Height

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `uav_waypoints.yaml`

### 2. Drone Group Communication Topology Initialization

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `params.yaml`

### 3. Obstacle Coordinates, Radius, Type (Static/Dynamic)

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `obstacles.yaml`

### 4. Formation Related Parameters

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `formation_config.yaml`

---

## 3. Core Project Functions (Example: `CONTROL_MODE == 1`)

### 1. Drone Waiting for Command Stage

### 2. Drone Group Initialization and Trajectory Pre-planning

- Open a new terminal:
    ```bash
    rostopic pub /mission_start std_msgs/Bool "data: true"
    ```
    - All drones enter obstacle parameter loading and trajectory planning state, waiting for takeoff command.

### 3. Drone Group Takeoff Stage

- Open a new terminal:
    ```bash
    rostopic pub /terminal_takeoff std_msgs/Bool "data: true"
    ```
    - All drones take off to the specified altitude, hover after reaching the altitude, and await task instructions.

### 4. Drone Group Task Execution Stage (Workspace Example: `wjx_ws`)

- Open a new terminal:
    ```bash
    cd ~/wjx_ws/src/offboard_pkg/scripts/
    ./start_dis_ctl_and_dyn_obs.sh
    ```
    - Drones perform trajectory tracking under MPC safety control, while dynamic obstacles start moving.

### 5. Restart Task After Completion

- Open a new terminal:
    ```bash
    rostopic pub /new_goal geometry_msgs/Point "x: 0.0  y: 0.0  z: 0.0"
    ```
    - Modify the target point as needed. Drones will fly to the preset target and hover after arrival.

### 6. Drone Formation Change

- Default formation is circular. Switch between circle and line formations:
    - Circle → Line:
        ```bash
        rostopic pub /formation_change std_msgs/String "data: 'line'"
        ```
    - Line → Circle:
        ```bash
        rostopic pub /formation_change std_msgs/String "data: 'circle'"
        ```
    - Drones will transform formation and hover at the designated position.

### 7. Simulate Node Failure to Trigger Reduced Member Formation Change (Example: `uav1`)

- Open a new terminal:
    ```bash
    rostopic pub /uav1/emergency std_msgs/Bool "data: true"
    ```
    - Drones detect disabled nodes and automatically change formation according to the number of online members.

### 8. Simulate Node Reload to Trigger Increased Member Formation Change (Example: `uav1`)

- Open a new terminal:
    ```bash
    rostopic pub /uav1/mission_start std_msgs/Bool "data: true"
    ```
    - Online drones detect reloaded nodes and trigger formation change according to the current number of online members.

    ```bash
    rostopic pub /uav1/terminal_takeoff std_msgs/Bool "data: true"
    ```
    - Reloaded drones will take off and hover at the group position, maintaining hover after reaching the designated spot.

---

**End of Instructions**
