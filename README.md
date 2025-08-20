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
     <img width="1009" height="654" alt="image" src="https://github.com/user-attachments/assets/ccbb6e7f-1029-4639-9d53-84a7c93bed8e" />

4. In Terminal 1:
    ```bash
    rosrun sim_rc sim_rc_node.py
    ```
    - Terminal 3 displays: `[PX4CTRL] RC received`
    <img width="1012" height="616" alt="image" src="https://github.com/user-attachments/assets/cc5bafef-3fee-4769-9a67-146e5884a886" />
5. Open Terminal 4:
    ```bash
    roslaunch px4 multi_mavros_ten_circle.launch
    ```
    - Wait for Terminal 3 to display the drone battery status
    <img width="1011" height="643" alt="image" src="https://github.com/user-attachments/assets/043c8b15-00d7-4a0c-816d-6a2dc136445e" />

6. In Terminal 1:
    ```
    5 2000
    6 2000
    ```
    - Terminal 3 indicates ready for takeoff
    <img width="763" height="490" alt="image" src="https://github.com/user-attachments/assets/bd31aebb-6595-400f-9a17-2951b4675b35" />
    
7. In Terminal 2:
    ```bash
    ./takeoff.sh
    ```
    - Terminal 3 shows drones entering `AUTO_TAKEOFF` mode
    <img width="756" height="502" alt="image" src="https://github.com/user-attachments/assets/b758caa7-9ef1-4792-8003-86eb808cd13c" />

8. In Terminal 1:
    ```
    7 2000
    ```
    - Terminal 3 shows drones entering `CMD_CTRL` mode
    <img width="762" height="495" alt="image" src="https://github.com/user-attachments/assets/5fb42303-d7e4-4d3d-a8a3-17aac751e0e0" />

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
  
   <img width="447" height="423" alt="image" src="https://github.com/user-attachments/assets/ba5cf700-6403-4d10-a1d9-8d1d4eaf98c2" />

### 2. Drone Group Communication Topology Initialization

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `params.yaml`
  
  <img width="534" height="605" alt="image" src="https://github.com/user-attachments/assets/14123eed-4803-495b-bc9f-b194cc171c6d" />


### 3. Obstacle Coordinates, Radius, Type (Static/Dynamic)

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `obstacles.yaml`
  
  <img width="648" height="420" alt="image" src="https://github.com/user-attachments/assets/722a51ad-d8fd-4a9d-be5d-ab98e58cb079" />


### 4. Formation Related Parameters

- File Directory: `/home/wjx/wjx_ws/src/offboard_pkg/config`
- File Name: `formation_config.yaml`
  
  <img width="831" height="432" alt="image" src="https://github.com/user-attachments/assets/347adbf3-89dc-43ff-9e51-a3eb448df80d" />

---

## 3. Core Project Functions (Example: `CONTROL_MODE == 1`)

### 1. Drone Waiting for Command Stage

### 2. Drone Group Initialization and Trajectory Pre-planning

- Open a new terminal:
    ```bash
    rostopic pub /mission_start std_msgs/Bool "data: true"
    ```
    - All drones enter obstacle parameter loading and trajectory planning state, waiting for takeoff command.
      
  <img width="1524" height="1095" alt="image" src="https://github.com/user-attachments/assets/349ae691-0fdb-48c9-9615-58920cd85691" />


### 3. Drone Group Takeoff Stage

- Open a new terminal:
    ```bash
    rostopic pub /terminal_takeoff std_msgs/Bool "data: true"
    ```
    - All drones take off to the specified altitude, hover after reaching the altitude, and await task instructions.
      
  <img width="1584" height="1095" alt="image" src="https://github.com/user-attachments/assets/2cf7adf9-a3f1-4c6b-9548-85b3dbc0d1c2" />


### 4. Drone Group Task Execution Stage (Workspace Example: `wjx_ws`)

- Open a new terminal:
    ```bash
    cd ~/wjx_ws/src/offboard_pkg/scripts/
    ./start_dis_ctl_and_dyn_obs.sh
    ```
    - Drones perform trajectory tracking under MPC safety control, while dynamic obstacles start moving.
      
  <img width="1584" height="1110" alt="image" src="https://github.com/user-attachments/assets/9693f50f-f773-4d0d-aabf-4f0924c31b52" />

### 5. Restart Task After Completion

- Open a new terminal:
    ```bash
    rostopic pub /new_goal geometry_msgs/Point "x: 0.0  y: 0.0  z: 0.0"
    ```
    - Modify the target point as needed. Drones will fly to the preset target and hover after arrival.
      
  <img width="1560" height="1113" alt="image" src="https://github.com/user-attachments/assets/6e1f3b46-5ad7-4490-9b34-f15b69ec3fd2" />


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
      
  <img width="1569" height="1113" alt="image" src="https://github.com/user-attachments/assets/de89d7e7-5ac2-4f20-b808-785d819190b6" />

### 7. Simulate Node Failure to Trigger Reduced Member Formation Change (Example: `uav1`)

- Open a new terminal:
    ```bash
    rostopic pub /uav1/emergency std_msgs/Bool "data: true"
    ```
    - Drones detect disabled nodes and automatically change formation according to the number of online members.
      
  <img width="1548" height="1116" alt="image" src="https://github.com/user-attachments/assets/a681f2f4-279d-4f03-b70d-38c3e1051203" />


### 8. Simulate Node Reload to Trigger Increased Member Formation Change (Example: `uav1`)

- Open a new terminal:
    ```bash
    rostopic pub /uav1/mission_start std_msgs/Bool "data: true"
    ```
    - Online drones detect reloaded nodes and trigger formation change according to the current number of online members.
      
  <img width="1539" height="1104" alt="image" src="https://github.com/user-attachments/assets/9849ca37-63f5-4f87-b24d-06ec219aa0da" />

    ```bash
    rostopic pub /uav1/terminal_takeoff std_msgs/Bool "data: true"
    ```
    - Reloaded drones will take off and hover at the group position, maintaining hover after reaching the designated spot.
      
  <img width="1572" height="1107" alt="image" src="https://github.com/user-attachments/assets/350305f9-fdd7-4f93-b418-4b0b350b34fe" />

---

**End of Instructions**
