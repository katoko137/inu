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

## 図の色分け

全フロー図で、同じ色を同じROS 2要素に使用する。

| 色 | 区分 | 意味・例 |
|---|---|---|
| 青 | ROS 2ノード | `bt_navigator`、`planner_server`、`go2_base` |
| 緑 | ROS 2トピック | `/cmd_vel`、`/scan`、`/api/sport/request` |
| 黄 | ROS 2メッセージ型・データ | `geometry_msgs/msg/Twist`、`nav_msgs/msg/Path` |
| 紫 | ROS 2 Action | `/navigate_to_pose`、`/compute_path_to_pose`、`/follow_path` |
| ピンク | TF・座標変換 | `map → odom`、`odom → base_link` |
| 灰 | 内部処理・プラグイン | Behavior Tree、Navfn、DWB、costmap |
| オレンジ | 操作者・Go2実機 | 操作者、Go2内蔵モーション制御 |

```mermaid
flowchart LR
    L_NODE["ROS 2ノード"]
    L_TOPIC["ROS 2トピック"]
    L_MSG["メッセージ型・データ"]
    L_ACTION["ROS 2 Action"]
    L_TF["TF・座標変換"]
    L_PROCESS["内部処理・プラグイン"]
    L_EXTERNAL["操作者・Go2実機"]

    classDef rosNode fill:#d6eaff,stroke:#2471a3,color:#102a43,stroke-width:2px;
    classDef rosTopic fill:#d5f5e3,stroke:#1e8449,color:#123524,stroke-width:2px;
    classDef rosMsg fill:#fcf3cf,stroke:#b7950b,color:#4d3d00,stroke-width:2px;
    classDef rosAction fill:#eadcf8,stroke:#7d3c98,color:#321442,stroke-width:2px;
    classDef tfData fill:#fadbd8,stroke:#c0392b,color:#4a1712,stroke-width:2px;
    classDef process fill:#e5e7e9,stroke:#5d6d7e,color:#1f2933,stroke-width:2px;
    classDef external fill:#fdebd0,stroke:#ca6f1e,color:#512e0b,stroke-width:2px;

    class L_NODE rosNode;
    class L_TOPIC rosTopic;
    class L_MSG rosMsg;
    class L_ACTION rosAction;
    class L_TF tfData;
    class L_PROCESS process;
    class L_EXTERNAL external;
```

## メインフロー

```mermaid
flowchart TD
    USER["操作者<br/>RViz上でNav2 Goalを指定"]

    subgraph RVIZ["RViz 2"]
        RVIZ_NODE["ノード: rviz2"]
        GOAL_TOOL["内部処理: nav2_rviz_plugins/GoalTool"]
    end

    subgraph NAV2["Nav2"]
        NAV_ACTION["Action: /navigate_to_pose"]
        NAV_MSG["Action型: nav2_msgs/action/NavigateToPose"]
        BTN["ノード: bt_navigator"]
        BT["内部処理: Behavior Tree<br/>navigate_w_replanning_and_recovery.xml"]
        COMPUTE_ACTION["Action: /compute_path_to_pose"]
        COMPUTE_MSG["Action型: nav2_msgs/action/ComputePathToPose"]
        PS["ノード: planner_server"]
        NAVFN["プラグイン: GridBased / NavfnPlanner"]
        GC["内部データ: global_costmap<br/>map座標系"]
        PLAN_TOPIC["トピック: /plan"]
        PATH_MSG["メッセージ: nav_msgs/msg/Path"]
        FOLLOW_ACTION["Action: /follow_path"]
        FOLLOW_MSG["Action型: nav2_msgs/action/FollowPath"]
        CS["ノード: controller_server"]
        DWB["プラグイン: FollowPath / DWBLocalPlanner<br/>20 Hz"]
        LC["内部データ: local_costmap<br/>odom座標系"]
        CMD_TOPIC["トピック: /cmd_vel"]
        TWIST_MSG["メッセージ: geometry_msgs/msg/Twist"]
    end

    subgraph BRIDGE["Go2 ROS 2制御"]
        BASE["ノード: go2_base"]
        HANDLE["内部処理: handleVelocity()"]
        SPORT["内部処理: SportClient::Move()<br/>Move API ID: 1008"]
        SPORT_TOPIC["トピック: /api/sport/request"]
        REQUEST_MSG["メッセージ: unitree_api/msg/Request<br/>api_id=1008 / parameter={x,y,z}"]
    end

    subgraph ROBOT["Go2本体"]
        DDS["Go2実機: Unitree DDS / Sport API"]
        MOTION["Go2内蔵モーション制御"]
        MOVE["脚の歩行開始"]
    end

    USER --> RVIZ_NODE
    RVIZ_NODE --> GOAL_TOOL
    GOAL_TOOL --> NAV_ACTION
    NAV_MSG -.-> NAV_ACTION
    NAV_ACTION --> BTN
    BTN --> BT

    BT --> COMPUTE_ACTION
    COMPUTE_MSG -.-> COMPUTE_ACTION
    COMPUTE_ACTION -->|goal + planner_id=GridBased| PS
    PS --> NAVFN
    GC -->|障害物・地図コスト| PS
    PS --> PLAN_TOPIC
    PLAN_TOPIC --> PATH_MSG
    PATH_MSG -->|Action result内のpath| BT

    BT --> FOLLOW_ACTION
    FOLLOW_MSG -.-> FOLLOW_ACTION
    FOLLOW_ACTION -->|path + controller_id=FollowPath| CS
    PATH_MSG -.->|RViz表示用| RVIZ_NODE
    LC -->|近傍障害物コスト| CS
    CS --> DWB
    DWB --> CMD_TOPIC
    CMD_TOPIC --> TWIST_MSG

    TWIST_MSG --> BASE
    BASE --> HANDLE
    HANDLE --> SPORT
    SPORT --> SPORT_TOPIC
    SPORT_TOPIC --> REQUEST_MSG
    REQUEST_MSG --> DDS
    DDS --> MOTION
    MOTION --> MOVE

    classDef rosNode fill:#d6eaff,stroke:#2471a3,color:#102a43,stroke-width:2px;
    classDef rosTopic fill:#d5f5e3,stroke:#1e8449,color:#123524,stroke-width:2px;
    classDef rosMsg fill:#fcf3cf,stroke:#b7950b,color:#4d3d00,stroke-width:2px;
    classDef rosAction fill:#eadcf8,stroke:#7d3c98,color:#321442,stroke-width:2px;
    classDef process fill:#e5e7e9,stroke:#5d6d7e,color:#1f2933,stroke-width:2px;
    classDef external fill:#fdebd0,stroke:#ca6f1e,color:#512e0b,stroke-width:2px;

    class RVIZ_NODE,BTN,PS,CS,BASE rosNode;
    class PLAN_TOPIC,CMD_TOPIC,SPORT_TOPIC rosTopic;
    class NAV_MSG,COMPUTE_MSG,FOLLOW_MSG,PATH_MSG,TWIST_MSG,REQUEST_MSG rosMsg;
    class NAV_ACTION,COMPUTE_ACTION,FOLLOW_ACTION rosAction;
    class GOAL_TOOL,BT,NAVFN,GC,DWB,LC,HANDLE,SPORT process;
    class USER,DDS,MOTION,MOVE external;
```

## 時系列フロー

Mermaidのシーケンス図はフローチャートと同じノード単位の色分けが難しいため、各名称に`[Action]`、`[トピック]`、`[メッセージ]`などの種別を明記している。

```mermaid
sequenceDiagram
    autonumber
    actor User as 操作者
    participant RViz as ノード: rviz2
    participant BT as ノード: bt_navigator
    participant Planner as ノード: planner_server
    participant GCost as 内部データ: global_costmap
    participant Controller as ノード: controller_server
    participant LCost as 内部データ: local_costmap
    participant Base as ノード: go2_base
    participant Sport as 内部処理: SportClient
    participant Go2 as 実機: Go2本体

    User->>RViz: 地図上で目標位置と向きを指定
    RViz->>BT: [Action] /navigate_to_pose Goal
    Note over RViz,BT: [Action型] nav2_msgs/action/NavigateToPose<br/>目標は通常map座標系

    BT->>Planner: [Action] /compute_path_to_pose Goal
    Note over BT,Planner: planner_id = GridBased
    Planner->>GCost: 現在位置・目標位置・コストを参照
    GCost-->>Planner: global costmap
    Planner->>Planner: NavfnPlannerでグローバル経路を計算
    Planner-->>BT: [メッセージ] nav_msgs/msg/Path
    Planner-->>RViz: [トピック] /planをpublish

    BT->>Controller: [Action] /follow_path Goal
    Note over BT,Controller: controller_id = FollowPath<br/>PathメッセージをAction Goal内で渡す

    loop 制御周期 20 Hz
        Controller->>LCost: ロボット周辺の障害物コストを参照
        LCost-->>Controller: local costmap
        Controller->>Controller: DWBで候補軌道を評価
        Controller->>Base: [トピック] /cmd_vel<br/>[メッセージ] geometry_msgs/msg/Twist
    end

    Base->>Base: Twistからvx, vy, vyawを取得
    Base->>Sport: SportClient::Move(req, vx, vy, vyaw)
    Sport->>Go2: [トピック] /api/sport/request<br/>[メッセージ] unitree_api/msg/Request<br/>Move API ID 1008
    Go2->>Go2: 速度指令を内蔵歩行制御へ反映
    Go2-->>User: 実際の歩行開始
```

## 経路計画・制御に必要な入力

```mermaid
flowchart LR
    subgraph SENSOR["Go2センサー入力"]
        GO2_LIDAR["Go2実機: LiDAR"]
        LIDAR_CLOUD["トピック: /utlidar/cloud_deskewed"]
        CLOUD_MSG["メッセージ: sensor_msgs/msg/PointCloud2"]
        LIDAR_POSE["トピック: /utlidar/robot_pose"]
        POSE_MSG["メッセージ: geometry_msgs/msg/PoseStamped"]
    end

    subgraph PERCEPTION["点群・LaserScan変換"]
        ACC["ノード: cloud_accumulation"]
        TRANS_CLOUD["トピック: /trans_cloud"]
        P2L["ノード: pointcloud_to_laserscan_node"]
        SCAN["トピック: /scan"]
        SCAN_MSG["メッセージ: sensor_msgs/msg/LaserScan"]
    end

    subgraph LOCALIZATION["自己位置・TF"]
        BASE["ノード: go2_base"]
        ODOM["トピック: /odom"]
        ODOM_MSG["メッセージ: nav_msgs/msg/Odometry"]
        TF_OB["TF: odom → base_link"]
        SLAM["ノード: slam_toolbox"]
        MAP["トピック: /map と /map_updates"]
        MAP_MSG["メッセージ: nav_msgs/msg/OccupancyGrid"]
        TF_MO["TF: map → odom"]
    end

    subgraph COSTMAPS["Nav2コストマップ"]
        GLOBAL["内部データ: global_costmap<br/>static + obstacle + inflation"]
        LOCAL["内部データ: local_costmap<br/>voxel + inflation"]
    end

    subgraph SERVERS["Nav2サーバー"]
        PLANNER["ノード: planner_server"]
        CONTROLLER["ノード: controller_server"]
    end

    GO2_LIDAR --> LIDAR_CLOUD
    LIDAR_CLOUD --> CLOUD_MSG
    CLOUD_MSG --> ACC
    ACC --> TRANS_CLOUD
    TRANS_CLOUD --> P2L
    P2L --> SCAN
    SCAN --> SCAN_MSG

    GO2_LIDAR --> LIDAR_POSE
    LIDAR_POSE --> POSE_MSG
    POSE_MSG --> BASE
    BASE --> ODOM
    ODOM --> ODOM_MSG
    BASE --> TF_OB

    SCAN_MSG --> SLAM
    ODOM_MSG --> SLAM
    TF_OB --> SLAM
    SLAM --> MAP
    MAP --> MAP_MSG
    SLAM --> TF_MO

    MAP_MSG --> GLOBAL
    SCAN_MSG --> GLOBAL
    TF_MO --> GLOBAL
    TF_OB --> GLOBAL

    SCAN_MSG --> LOCAL
    TF_OB --> LOCAL

    GLOBAL --> PLANNER
    TF_MO --> PLANNER
    TF_OB --> PLANNER

    LOCAL --> CONTROLLER
    ODOM_MSG --> CONTROLLER
    TF_OB --> CONTROLLER

    classDef rosNode fill:#d6eaff,stroke:#2471a3,color:#102a43,stroke-width:2px;
    classDef rosTopic fill:#d5f5e3,stroke:#1e8449,color:#123524,stroke-width:2px;
    classDef rosMsg fill:#fcf3cf,stroke:#b7950b,color:#4d3d00,stroke-width:2px;
    classDef tfData fill:#fadbd8,stroke:#c0392b,color:#4a1712,stroke-width:2px;
    classDef process fill:#e5e7e9,stroke:#5d6d7e,color:#1f2933,stroke-width:2px;
    classDef external fill:#fdebd0,stroke:#ca6f1e,color:#512e0b,stroke-width:2px;

    class ACC,P2L,BASE,SLAM,PLANNER,CONTROLLER rosNode;
    class LIDAR_CLOUD,LIDAR_POSE,TRANS_CLOUD,SCAN,ODOM,MAP rosTopic;
    class CLOUD_MSG,POSE_MSG,SCAN_MSG,ODOM_MSG,MAP_MSG rosMsg;
    class TF_OB,TF_MO tfData;
    class GLOBAL,LOCAL process;
    class GO2_LIDAR external;
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
    START["Action: NavigateToPose開始"]
    PLAN["Action: ComputePathToPose<br/>1 Hzで再計画"]
    PLAN_PROCESS["内部処理: Navfnで経路計画"]
    PATH_DATA["メッセージ: nav_msgs/msg/Path"]
    FOLLOW["Action: FollowPath"]
    CONTROL["内部処理: DWB制御"]
    CHECK{"内部判定: ゴール到達?"}
    DONE["Action結果: NavigateToPose成功"]
    FAIL{"計画・追従失敗?"}
    CLEAR_G["内部処理: global costmapをclear"]
    CLEAR_L["内部処理: local costmapをclear"]
    RECOVERY["全体Recovery<br/>costmap clear → Spin → Wait"]
    ABORT["Action結果: NavigateToPose失敗"]

    START --> PLAN
    PLAN --> PLAN_PROCESS
    PLAN_PROCESS --> PATH_DATA
    PATH_DATA --> FOLLOW
    FOLLOW --> CONTROL
    CONTROL --> CHECK
    CHECK -->|いいえ| PLAN
    CHECK -->|はい| DONE

    PLAN_PROCESS --> FAIL
    CONTROL --> FAIL
    FAIL -->|計画失敗| CLEAR_G
    FAIL -->|追従失敗| CLEAR_L
    CLEAR_G --> PLAN
    CLEAR_L --> FOLLOW
    FAIL -->|再試行上限| RECOVERY
    RECOVERY --> PLAN
    RECOVERY -->|Recovery上限超過| ABORT

    classDef rosMsg fill:#fcf3cf,stroke:#b7950b,color:#4d3d00,stroke-width:2px;
    classDef rosAction fill:#eadcf8,stroke:#7d3c98,color:#321442,stroke-width:2px;
    classDef process fill:#e5e7e9,stroke:#5d6d7e,color:#1f2933,stroke-width:2px;

    class START,PLAN,FOLLOW,DONE,ABORT rosAction;
    class PATH_DATA rosMsg;
    class PLAN_PROCESS,CONTROL,CHECK,FAIL,CLEAR_G,CLEAR_L,RECOVERY process;
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
