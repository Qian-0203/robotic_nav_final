# ROS Node and Topic Relation Diagram

This diagram is based on static inspection of this repo's ROS 2 launch files,
entry points, publishers, subscribers, action clients/servers, service
clients/servers, and Docker compose commands. External stacks are included only
where this repo connects to their topics, actions, or services.

## Main Mission / Perception / Control Graph

```mermaid
flowchart LR
  %% Sensors and external ROS stacks
  camera["Camera driver\n(astra/orbbec/unity)"]
  lidar["Lidar driver\n(rplidar/ydlidar/oradar/unity)"]
  imu["IMU driver\nwit_ros2_imu / pros_imu"]
  scan_matcher["scan_matcher\nros2_laser_scan_matcher"]
  slam["slam_toolbox"]
  nav2["Nav2\nlocalization/navigation"]
  rosbridge["rosbridge / foxglove_bridge"]
  remote_bridge["remote_topic_bridge"]

  %% Perception
  yolo["yolo_detection_node\npackage: yolo_pkg\nnode: RosCommunicator"]
  yolo_example["yolo_node\npackage: yolo_example_pkg\nlegacy/example"]
  aruco["arucode_node\nnode: aruco_detector"]
  depth_test["depth_test_node\nnode: depth_center_pixel_node"]

  %% Mission/control/action
  keyboard["keyboard_control_node\nnode: ros2_manager"]
  mission["mission_task_node"]
  mission_client["mission_task_client\nexternal CLI/client"]
  car_control["car_control_node\nBaseCarControlNode"]
  nav_action["navigation_action_server_node\naction: nav_action_server"]
  arm_control["arm_control_node\nArmCummuteNode"]
  arm_action["arm_action_server_node\naction: arm_action_server"]
  unity_arm["unity_arm_republish_node"]

  %% Low-level hardware bridge
  car_writer["carC_writer\ncar_c_control_subscriber"]
  car_reader["carC_reader\ncar_c_serial_reader"]
  arm_writer["arm_writer\narm_serial_writer"]
  arm_reader["arm_reader\narm_serial_reader"]
  robot_hw["Car / arm serial hardware"]

  %% Services and utility nodes
  data_collector["ros_car_communication_node\nCarROSCommunicator"]
  data_service["ros_car_server\nDataReceiverServiceNode"]
  nav_clients["robot_control / pros_car_py\nlegacy app client"]

  %% Camera/image topics
  camera -->|"/camera/image/compressed"| yolo
  camera -->|"/camera/depth/image_raw"| yolo
  camera -->|"/camera/depth/compressed"| yolo
  camera -->|"/out/compressed"| aruco
  camera -->|"/camera/depth/image_raw"| aruco
  camera -->|"/camera/image/compressed"| yolo_example
  camera -->|"/camera/depth/image_raw"| yolo_example
  camera -->|"/camera/depth/compressed"| yolo_example
  camera -->|"/unity_camera_1/depth or param depth_topic"| depth_test

  yolo -->|"/yolo/detection/compressed"| rosbridge
  yolo -->|"/yolo/detection/position"| nav_clients
  yolo -->|"/yolo/object/offset"| car_control
  yolo -->|"/yolo/object/offset"| arm_control
  yolo -->|"/yolo/object/offset"| mission
  yolo -->|"/yolo/detection/status"| nav_clients

  yolo_example -->|"/yolo/detection/compressed"| rosbridge
  yolo_example -->|"/yolo/segmentation/compressed"| rosbridge
  yolo_example -->|"/yolo/target_info"| nav_clients
  yolo_example -->|"/camera/x_multi_depth_values"| car_control
  yolo_example -->|"/camera/x_multi_depth_values"| nav_clients

  depth_test -->|"depth_center_value"| rosbridge
  aruco -->|"/aruco/id100/depth_m"| arm_control
  arm_control -->|"/aruco/id100/depth_m clear NaN"| arm_control

  mission -->|"/target_label"| yolo
  mission -->|"/goal_pose"| car_control
  mission -->|"/goal_pose"| nav2
  mission -->|"car_C_front_wheel"| car_writer
  mission -->|"car_C_rear_wheel"| car_writer
  mission -.->|"action client: nav_action_server"| nav_action
  mission -.->|"action client: arm_action_server"| arm_action
  mission -.->|"action server: mission_task_server"| mission_client

  keyboard -->|"car_control_signal"| car_control
  keyboard -->|"arm_control_signal"| arm_control
  keyboard -.->|"action client: nav_action_server"| nav_action
  keyboard -.->|"action client: arm_action_server"| arm_action

  car_control -->|"car_C_front_wheel"| car_writer
  car_control -->|"car_C_rear_wheel"| car_writer
  car_control -->|"/goal_pose clear"| nav2
  car_control -->|"/plan clear"| nav2
  car_control -->|"latest nav/perception state"| nav_action
  nav_action -.->|"action client: compute_path_to_pose"| nav2

  arm_control -->|"/robot_arm"| unity_arm
  arm_control -->|"/robot_arm"| arm_writer
  arm_control -->|"car_C_front_wheel"| car_writer
  arm_control -->|"car_C_rear_wheel"| car_writer
  arm_control -->|"goal_pose_tmp"| remote_bridge
  arm_control -->|"goal_marker"| rosbridge
  arm_control -->|"latest arm/perception state"| arm_action
  unity_arm -->|"/robot_arm"| arm_writer

  car_writer -->|serial writes| robot_hw
  robot_hw -->|serial reads| car_reader
  car_reader -->|"car_C_state"| data_collector
  car_reader -->|"car_C_state_front"| data_collector
  arm_writer -->|serial writes| robot_hw
  robot_hw -->|serial reads| arm_reader
  arm_reader -->|"joint_states"| rosbridge

  %% Nav/lidar/pose graph
  lidar -->|"/scan_tmp or /scan"| scan_matcher
  lidar -->|"/scan"| slam
  lidar -->|"/scan"| nav2
  scan_matcher -->|"/odom"| nav2
  scan_matcher -->|"/tf odom->base_footprint"| nav2
  slam -->|"/map, /tf"| nav2
  nav2 -->|"/amcl_pose"| car_control
  nav2 -->|"/amcl_pose"| arm_control
  nav2 -->|"/amcl_pose"| mission
  nav2 -->|"/amcl_pose"| data_collector
  nav2 -->|"/amcl_pose"| data_service
  nav2 -->|"/cmd_vel"| car_control
  nav2 -->|"/cmd_vel"| nav_clients
  nav2 -->|"/plan"| car_control
  nav2 -->|"/plan"| nav_clients
  nav2 -->|"/received_global_plan"| car_control
  nav2 -->|"/goal_pose"| data_collector
  nav2 -->|"/goal_pose"| data_service

  imu -->|"/imu/data"| yolo
  imu -->|"/imu/data"| nav_clients
  imu -->|"/imu/data_raw"| arm_control

  remote_bridge -->|"remote /goal_pose_tmp to local /goal_pose"| nav2
  data_collector -.->|"service: get_latest_data"| nav_clients
  data_service -.->|"services: get_latest_data, get_scan"| nav_clients
  nav_clients -.->|"service clients: /global_costmap/clear, /local_costmap/clear"| nav2
  nav_clients -.->|"action client: /navigate_to_pose"| nav2
```

## Compact Topic/Action Inventory

| Relation | Producers | Consumers |
| --- | --- | --- |
| `/camera/image/compressed` | camera driver / image transport | `yolo_detection_node`, `yolo_node`, `robot_control` legacy app |
| `/camera/color/image_raw` | camera driver / image transport | image transport republishers |
| `/out/compressed` | camera/image pipeline | `arucode_node` |
| `/camera/depth/image_raw` | camera driver | `yolo_detection_node`, `yolo_node`, `arucode_node` |
| `/camera/depth/compressed` | camera driver / image transport | `yolo_detection_node`, `yolo_node` |
| `/target_label` | `mission_task_node`, `robot_control` legacy app | `yolo_detection_node` |
| `/yolo/detection/compressed` | `yolo_detection_node`, `yolo_node` | visualization/bridge clients |
| `/yolo/object/offset` | `yolo_detection_node` | `car_control_node`, `arm_control_node`, `mission_task_node` |
| `/yolo/detection/position` | `yolo_detection_node` | `robot_control` legacy app |
| `/yolo/detection/status` | `yolo_detection_node` | `robot_control` legacy app |
| `/yolo/target_info` | `yolo_node` legacy/example | `robot_control` legacy app |
| `/camera/x_multi_depth_values` | `yolo_node` legacy/example | `car_control_node`, `robot_control` legacy app |
| `/aruco/id100/depth_m` | `arucode_node`, `arm_control_node` clear signal | `arm_control_node` |
| `/scan` | lidar/filter/unity driver | SLAM Toolbox, Nav2 costmaps, `car_control_node`, data service nodes, legacy app |
| `/scan_tmp` | rplidar/oradar raw driver | angle filter / scan matcher |
| `/odom` and `/tf` | scan matcher, robot state publisher, SLAM/Nav2 | Nav2, visualization clients |
| `/map` | SLAM Toolbox or map server | Nav2 |
| `/amcl_pose` | Nav2 localization | `car_control_node`, `arm_control_node`, `mission_task_node`, data service nodes, legacy app |
| `/goal_pose` | `mission_task_node`, `car_control_node`, remote bridge, legacy app | Nav2, `car_control_node`, data service nodes |
| `/move_base_simple/goal` | RViz/user tools | `car_control_node` |
| `/cmd_vel` | Nav2 controller | `car_control_node`, legacy app |
| `/plan` | Nav2 planner, `car_control_node` clear, legacy app | `car_control_node`, legacy app, visualization |
| `/received_global_plan` | legacy app | `car_control_node`, legacy app |
| `car_control_signal` | `keyboard_control_node` | `car_control_node` |
| `arm_control_signal` | `keyboard_control_node` | `arm_control_node` manual control |
| `car_C_front_wheel` | `mission_task_node`, `car_control_node`, `arm_control_node`, legacy app | `carC_writer` |
| `car_C_rear_wheel` | `mission_task_node`, `car_control_node`, `arm_control_node`, legacy app | `carC_writer` |
| `car_C_state`, `car_C_state_front` | `carC_reader` | downstream/debug consumers |
| `/robot_arm` | `arm_control_node`, `unity_arm_republish_node`, legacy app | `arm_writer`, Unity bridge |
| `joint_states` | `arm_reader` | robot state publisher / visualization |
| `goal_pose_tmp` | `arm_control_node` | `remote_topic_bridge` to local `/goal_pose` |
| `goal_marker` | `arm_control_node` | visualization clients |
| `nav_action_server` | server: `navigation_action_server_node` | clients: `keyboard_control_node`, `mission_task_node` |
| `arm_action_server` | server: `arm_action_server_node` | clients: `keyboard_control_node`, `mission_task_node` |
| `mission_task_server` | server: `mission_task_node` | mission task clients / keyboard or external caller |
| `compute_path_to_pose` | server: Nav2 planner | client: `navigation_action_server_node` |
| `/navigate_to_pose` | server: Nav2 behavior tree navigator | client: legacy `robot_control` app |
| `get_latest_data` | `ros_car_communication_node` or `ros_car_server` | service clients |
| `get_scan` | `ros_car_server` | service clients |
| `/global_costmap/clear`, `/local_costmap/clear` | Nav2 services | legacy `robot_control` app |

## Launch Entry Points Covered

- `workspace/pros/pros_car/src/mission_task_pkg/launch/task_bringup.launch.py`
  launches `car_control_node`, `arm_control_node`, `unity_arm_republish_node`, and
  `mission_task_node` for the mission stack.
- `workspace/pros/ros2_yolo_integration/src/yolo_pkg/launch/yolo_and_arucode.launch.py`
  launches `yolo_detection_node` and optionally `arucode_node`.
- `workspace/pros/pros_car/src/arm_control_pkg/launch/arm.launch.py` launches
  `arm_writer`, `arm_control_node`, and `unity_arm_republish_node`.
- `workspace/pros/pros_app/docker/compose/demo/*.xml` and `*.launch.py` wire
  external camera, lidar, SLAM, Nav2, rosbridge, and scan matcher components.

## Static Inspection Notes

- `car_control_pkg`, `arm_control_pkg`, and `mission_task_pkg` publish
  `std_msgs/Float32MultiArray` on `car_C_front_wheel` and `car_C_rear_wheel`.
  The legacy `pros_car_py.carC_serial_writer` subscribes as `std_msgs/String` and
  expects JSON. If that writer is used with the newer control packages, the topic
  names match but the message types do not.
- `arucode_node` subscribes to `/out/compressed`, while the camera compose files
  mostly republish camera streams under `/camera/...`. A remap or republisher is
  needed for ArUco in that setup.
