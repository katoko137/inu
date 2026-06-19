# RVizからNav2経由でGo2が動き出すまでの処理フロー

## 対象構成

この資料は、次の起動構成を対象とする。

```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/install/setup.bash
source ~/go2_ros2_ws/install/setup.bash
ros2 launch go2_core go2_startup.launch.py
```

`go2_startup.launch.py`から、主に以下が起動される。

- `rviz2`
- `go2_base`
- `cloud_accumulation`
- `pointcloud_to_laserscan_node`
- `slam_toolbox`
- Nav2の`planner_server`
- Nav2の`controller_server`
- Nav2の`recoveries_server`
- Nav2の`bt_navigator`
- Nav2の`waypoint_follower`
- `lifecycle_manager_navigation`

Nav2パラメータは、実行時に以下のファイルが使用される。

```text
/home/unitree/go2_ros2_ws/install/go2_navigation/share/go2_navigation/config/nav2_params.yaml
```

## メインフロー

```mermaid
flowchart TD
    USER["操作者<br/>RViz上でNav2 Goalを指定"]

    subgraph RVIZ["RViz 2"]
        GOAL_TOOL["nav2_rviz_plugins/GoalTool<br/>目標位置・目標姿勢を作成"]
    end

    subgraph NAV2["Nav2"]
        BTN["bt_navigator<br/>Behavior Tree実行"]
        BT["navigate_w_replanning_and_recovery.xml<br/>経路計画と追従を管理"]
        PS["planner_server<br/>GridBased: NavfnPlanner"]
        GC["global_costmap<br/>map座標系"]
        PATH["グローバル経路<br/>nav_msgs/Path"]
        CS["controller_server<br/>FollowPath: DWBLocalPlanner<br/>20 Hz"]
        LC["local_costmap<br/>odom座標系"]
        CMD["速度指令<br/>geometry_msgs/Twist"]
    end

    subgraph BRIDGE["Go2 ROS 2制御"]
        BASE["go2_base<br/>handleVelocity()"]
        SPORT["SportClient::Move()<br/>Move API ID: 1008"]
    end

    subgraph ROBOT["Go2本体"]
        DDS["Unitree DDS / Sport API"]
        MOTION["Go2内蔵モーション制御"]
        MOVE["脚の歩行開始"]
    end

    USER --> GOAL_TOOL
    GOAL_TOOL -->|"/navigate_to_pose Action<br/>nav2_msgs/action/NavigateToPose"| BTN
    BTN --> BT

    BT -->|"/compute_path_to_pose Action<br/>goal + planner_id=GridBased"| PS
    GC -->|障害物・地図コスト| PS
    PS -->|"/plan<br/>nav_msgs/msg/Path"| PATH
    PS -->|Action result内のpath| BT

    BT -->|"/follow_path Action<br/>path + controller_id=FollowPath"| CS
    PATH -.->|RViz表示用| GOAL_TOOL
    LC -->|近傍障害物コスト| CS
    CS -->|"/cmd_vel<br/>geometry_msgs/msg/Twist"| CMD

    CMD --> BASE
    BASE --> SPORT
    SPORT -->|"/api/sport/request<br/>unitree_api/msg/Request<br/>api_id=1008<br/>parameter={x,y,z}"| DDS
    DDS --> MOTION
    MOTION --> MOVE
```

## 時系列フロー

```mermaid
sequenceDiagram
    autonumber
    actor User as 操作者
    participant RViz as rviz2 GoalTool
    participant BT as bt_navigator
    participant Planner as planner_server
    participant GCost as global_costmap
    participant Controller as controller_server
    participant LCost as local_costmap
    participant Base as go2_base
    participant Sport as Unitree Sport API
    participant Go2 as Go2本体

    User->>RViz: 地図上で目標位置と向きを指定
    RViz->>BT: /navigate_to_pose Action Goal
    Note over RViz,BT: 型: nav2_msgs/action/NavigateToPose<br/>目標は通常map座標系

    BT->>Planner: /compute_path_to_pose Action Goal
    Note over BT,Planner: planner_id = GridBased
    Planner->>GCost: 現在位置・目標位置・コストを参照
    GCost-->>Planner: global costmap
    Planner->>Planner: NavfnPlannerでグローバル経路を計算
    Planner-->>BT: 計算済みPath
    Planner-->>RViz: /planをpublish

    BT->>Controller: /follow_path Action Goal
    Note over BT,Controller: controller_id = FollowPath<br/>PathをAction Goal内で渡す

    loop 制御周期 20 Hz
        Controller->>LCost: ロボット周辺の障害物コストを参照
        LCost-->>Controller: local costmap
        Controller->>Controller: DWBで候補軌道を評価
        Controller->>Base: /cmd_velをpublish
    end

    Base->>Base: Twistからvx, vy, vyawを取得
    Base->>Sport: SportClient::Move(req, vx, vy, vyaw)
    Sport->>Go2: /api/sport/request<br/>Move API ID 1008
    Go2->>Go2: 速度指令を内蔵歩行制御へ反映
    Go2-->>User: 実際の歩行開始
```

## 経路計画・制御に必要な入力

```mermaid
flowchart LR
    subgraph SENSOR["Go2センサー入力"]
        LIDAR_CLOUD["/utlidar/cloud_deskewed<br/>sensor_msgs/PointCloud2"]
        LIDAR_POSE["/utlidar/robot_pose<br/>geometry_msgs/PoseStamped"]
    end

    subgraph PERCEPTION["点群・LaserScan変換"]
        ACC["cloud_accumulation"]
        P2L["pointcloud_to_laserscan_node"]
        SCAN["/scan<br/>sensor_msgs/LaserScan"]
    end

    subgraph LOCALIZATION["自己位置・TF"]
        BASE["go2_base"]
        ODOM["/odom<br/>nav_msgs/Odometry"]
        TF_OB["TF: odom → base_link"]
        SLAM["slam_toolbox"]
        MAP["/map と /map_updates<br/>nav_msgs/OccupancyGrid"]
        TF_MO["TF: map → odom"]
    end

    subgraph COSTMAPS["Nav2コストマップ"]
        GLOBAL["global_costmap<br/>static + obstacle + inflation"]
        LOCAL["local_costmap<br/>voxel + inflation"]
    end

    subgraph SERVERS["Nav2サーバー"]
        PLANNER["planner_server"]
        CONTROLLER["controller_server"]
    end

    LIDAR_CLOUD --> ACC
    ACC -->|"/trans_cloud"| P2L
    P2L --> SCAN

    LIDAR_POSE --> BASE
    BASE --> ODOM
    BASE --> TF_OB

    SCAN --> SLAM
    ODOM --> SLAM
    TF_OB --> SLAM
    SLAM --> MAP
    SLAM --> TF_MO

    MAP --> GLOBAL
    SCAN --> GLOBAL
    TF_MO --> GLOBAL
    TF_OB --> GLOBAL

    SCAN --> LOCAL
    TF_OB --> LOCAL

    GLOBAL --> PLANNER
    TF_MO --> PLANNER
    TF_OB --> PLANNER

    LOCAL --> CONTROLLER
    ODOM --> CONTROLLER
    TF_OB --> CONTROLLER
```

## ノードとインターフェース一覧

| 送信元 | インターフェース | 型 | 受信先・用途 |
|---|---|---|---|
| RViz `GoalTool` | `/navigate_to_pose` Action | `nav2_msgs/action/NavigateToPose` | `bt_navigator`へナビゲーション目標を送信 |
| `bt_navigator` | `/compute_path_to_pose` Action | `nav2_msgs/action/ComputePathToPose` | `planner_server`へ経路計画を要求 |
| `planner_server` | `/plan` Topic | `nav_msgs/msg/Path` | グローバル経路の可視化 |
| `bt_navigator` | `/follow_path` Action | `nav2_msgs/action/FollowPath` | `controller_server`へ経路追従を要求 |
| `controller_server` | `/cmd_vel` Topic | `geometry_msgs/msg/Twist` | `go2_base`へ速度指令を送信 |
| `go2_base` | `/api/sport/request` Topic | `unitree_api/msg/Request` | Go2のSport APIへ移動指令を送信 |
| Go2 LiDAR | `/utlidar/robot_pose` Topic | `geometry_msgs/msg/PoseStamped` | `go2_base`がオドメトリとTFへ変換 |
| `go2_base` | `/odom` Topic | `nav_msgs/msg/Odometry` | `bt_navigator`、`controller_server`、SLAMが使用 |
| `go2_base` | `/tf` | `tf2_msgs/msg/TFMessage` | `odom → base_link`を配信 |
| Go2 LiDAR | `/utlidar/cloud_deskewed` Topic | `sensor_msgs/msg/PointCloud2` | 点群処理の入力 |
| `cloud_accumulation` | `/trans_cloud` Topic | `sensor_msgs/msg/PointCloud2` | LaserScan変換の入力 |
| `pointcloud_to_laserscan_node` | `/scan` Topic | `sensor_msgs/msg/LaserScan` | SLAM、global/local costmapが使用 |
| `slam_toolbox` | `/map` Topic | `nav_msgs/msg/OccupancyGrid` | global costmapが使用 |
| `slam_toolbox` | `/tf` | `tf2_msgs/msg/TFMessage` | `map → odom`を配信 |

## ROS 2 Actionの実体

`/navigate_to_pose`、`/compute_path_to_pose`、`/follow_path`は通常の単一トピックではなくROS 2 Actionである。各Actionは内部的に、概ね以下のサービスとトピックで構成される。

```text
/<action_name>/_action/send_goal
/<action_name>/_action/get_result
/<action_name>/_action/cancel_goal
/<action_name>/_action/feedback
/<action_name>/_action/status
```

例:

```text
/navigate_to_pose/_action/send_goal
/navigate_to_pose/_action/feedback
/navigate_to_pose/_action/status
```

## Behavior Treeの処理

使用されるBehavior TreeはNav2 Foxy標準の以下である。

```text
/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/
  navigate_w_replanning_and_recovery.xml
```

通常処理は次の順序になる。

```mermaid
flowchart TD
    START["NavigateToPose開始"]
    PLAN["ComputePathToPose<br/>1 Hzで再計画"]
    FOLLOW["FollowPath<br/>DWB制御"]
    CHECK{"ゴール到達?"}
    DONE["NavigateToPose成功"]
    FAIL{"計画・追従失敗?"}
    CLEAR_G["global costmapをclear"]
    CLEAR_L["local costmapをclear"]
    RECOVERY["全体Recovery<br/>costmap clear → Spin → Wait"]
    ABORT["NavigateToPose失敗"]

    START --> PLAN
    PLAN --> FOLLOW
    FOLLOW --> CHECK
    CHECK -->|いいえ| PLAN
    CHECK -->|はい| DONE

    PLAN --> FAIL
    FOLLOW --> FAIL
    FAIL -->|計画失敗| CLEAR_G
    FAIL -->|追従失敗| CLEAR_L
    CLEAR_G --> PLAN
    CLEAR_L --> FOLLOW
    FAIL -->|再試行上限| RECOVERY
    RECOVERY --> PLAN
    RECOVERY -->|Recovery上限超過| ABORT
```

## Go2が動き始める成立条件

Go2が実際に歩き始めるのは、以下がすべて成立した後である。

1. `bt_navigator`が`/navigate_to_pose` Goalを受理する。
2. `map → odom → base_link`のTFが解決できる。
3. `planner_server`が有効なグローバル経路を生成する。
4. `controller_server`が`/follow_path`を受理する。
5. local costmapと現在姿勢を使い、DWBが有効な速度候補を選択する。
6. `controller_server`がゼロではない`/cmd_vel`をpublishする。
7. `go2_base`が`/cmd_vel`をMove API ID `1008`へ変換する。
8. Go2本体のSport Modeが指令を受け付ける。

`go2_base`は速度値を次のように変換する。

```text
/cmd_vel.linear.x  → parameter.x
/cmd_vel.linear.y  → parameter.y
/cmd_vel.angular.z → parameter.z
```

3軸がすべて厳密にゼロの場合、Move APIに加えてStopMove API ID `1003`も送信する。

## 現在の主要パラメータ

実行時のinstall版設定では以下となっている。

| 項目 | 値 |
|---|---:|
| Controller | `dwb_core::DWBLocalPlanner` |
| Controller周期 | 20 Hz |
| Global planner | `nav2_navfn_planner/NavfnPlanner` |
| BT再計画周期 | 1 Hz |
| `max_vel_x` | 3.0 m/s |
| `max_vel_theta` | 1.5 rad/s |
| `min_speed_xy` | 0.3 m/s |
| Goal XY許容誤差 | 0.40 m |
| Goal yaw許容誤差 | 3.14 rad |
| local costmap | 3 m × 3 m、0.05 m/cell |
| ロボット半径 | 0.22 m |

## 動作確認用コマンド

実機を動かさずに接続状態を確認する場合:

```bash
ros2 action list
ros2 action info /navigate_to_pose
ros2 action info /compute_path_to_pose
ros2 action info /follow_path

ros2 topic info /cmd_vel
ros2 topic info /api/sport/request
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /map

ros2 node info /bt_navigator
ros2 node info /planner_server
ros2 node info /controller_server
ros2 node info /go2_base

ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

