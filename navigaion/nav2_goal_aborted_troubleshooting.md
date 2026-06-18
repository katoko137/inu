# ゴール送信直後に ABORTED が返る問題

作成日: 2026-06-18
対象環境: Unitree Go2 内部 Jetson（4コア）/ ROS2 Foxy / Nav2（RViz + Nav2 で移動制御）

---

## 1. 問題（Problem）

RViz 等でゴールを送信した**直後に `NavigateToPose` が ABORTED を返し**、Go2 が動かない。
ノード（planner_server 等）は生きており（lifecycle `active [3]`）、死活が原因ではない。
あわせて `bt_navigator` から **`send_goal failed`** も出ることがある。

---

## 2. 現象と確認したログ（Phenomenon / Evidence）

### (a) システム負荷 ＝ CPU 過負荷【本命】
実コマンド:
```bash
cat /proc/loadavg ; nproc ; free -h ; top -bn1
```
結果:
```
loadavg: 18.19 14.01 13.18      （CPUコア数: 4）  ← 4コアに対し負荷18 ≒ 4.5倍の過負荷
Mem: 15Gi中 9.2Gi available     ← メモリは余裕。ボトルネックは CPU
```
top の CPU 上位:
| プロセス | CPU |
|---|---|
| rviz2 | 70%（`/rviz2` が複数 = 多重起動の疑い） |
| pointcloud_to_laserscan | 55% |
| gnome-shell（GUI） | 40% |
| python3（あるノード） | 35% |
| async_slam_toolbox / cloud_accumulation | 15% / 10% |

### (b) controller_server が制御周期を維持できていない
`~/.ros/log/controller_server_17158_1781779006740.log`:
```
[WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz   ← 連続88回以上
[WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
```
→ DWB コントローラが設定の 20Hz で回れず、FollowPath が中断 → 上位で ABORTED。

### (c) bt_navigator の `send_goal failed`
`bt_navigator` の BT アクションノードは、各サーバへ `async_send_goal()` でゴールを送り、
**`server_timeout`（Foxy 既定 20ms）以内に受理応答(ACK)を待つ**。CPU 飢餓でサーバが
20ms 以内に ACK を返せないとタイムアウトし、以下が投げられる:
```cpp
if (spin_until_future_complete(future, server_timeout_) != SUCCESS)
    throw std::runtime_error("send_goal failed");
```
→ 設定（`nav2_params.yaml` の `bt_navigator`）に `server_timeout` / `bt_loop_duration`
の明示指定が無く **既定値（20ms / 10ms）** のため、過負荷時に真っ先に出る。

### (d) グローバルコストマップが頻繁にリサイズ
`~/.ros/log/planner_server_17163_1781779006752.log`:
```
StaticLayer: Resizing costmap to 67 X 212 ... → 148 X 345 ...（SLAM地図の拡大に伴い連続）
```
→ SLAM(`async_slam_toolbox`)地図の成長に合わせ costmap が再構築され続け、さらに CPU を消費。

> 補足: 過去ログ `~/.ros/log/planner_server_14342_*.log` には
> 「Robot is out of bounds of the costmap」が多発していた時期もあった（地図とロボット位置の
> 不整合）。ただし現在は TF `map→odom→base_link` は解決でき、地図も 17m 級まで成長済み。

---

## 3. 原因と考えられる事項（Candidate Causes）

| # | 原因 | 確度 | 根拠 |
|---|------|------|------|
| 1 | **CPU 過負荷**（load 18 / 4コア） | ★★★ 本命 | (a)(b)(c) すべてが CPU 飢餓の症状 |
| 2 | controller の制御ループ破綻（20Hz 未達）→ FollowPath abort | ★★★ | (b) Control loop missed 連発 + Aborting handle |
| 3 | bt_navigator の `server_timeout`=20ms が短すぎ → send_goal failed | ★★☆ | (c) 既定値のまま |
| 4 | SLAM 地図の継続リサイズによる負荷・一時的な範囲外 | ★★☆ | (d) costmap 連続 Resizing |
| 5 | RViz の多重起動・Jetson 上での描画（70% CPU） | ★★☆ | top + ノード一覧に `/rviz2` 複数 |
| 6 | （過去要因）コストマップ範囲外 = 地図とロボット位置の不整合 | ★☆☆ | 旧ログの out of bounds。現在は解消傾向 |

→ **結論: 直接の引き金は「CPU 過負荷」**。ABORTED（実行中失敗）も send_goal failed
（受理段階失敗）も、根は同一。

---

## 4. 緩和策（Mitigations）

### A. CPU 負荷を下げる【最優先・本命対策】
1. **RViz を Jetson 上で動かさない**。
   - NoMachine/VNC/X転送は **画面転送なので RViz プロセスは Jetson 上に残り効果なし**。
   - **リモートPCで RViz プロセスを起動**し、ROS2 DDS（CycloneDDS / eth0 / 同一 ROS_DOMAIN_ID）
     経由で購読させる。描画 CPU が丸ごとリモートへ移る。
   - `/rviz2` が複数起動していないか確認し、余分を閉じる。
2. **Nav2 の周期を下げる**:
   ```yaml
   controller_server:
     controller_frequency: 10.0       # 20.0 → 10.0
   planner_server:
     expected_planner_frequency: 5.0  # 20.0 → 5.0（NavFnに20Hzは過剰）
   ```
3. `pointcloud_to_laserscan` / SLAM の更新レート・点群密度を下げる。
4. GUI（gnome-shell）を使わず SSH / ヘッドレス運用にする。

### B. bt_navigator のタイムアウトを緩める【send_goal failed への即効策】
```yaml
bt_navigator:
  ros__parameters:
    server_timeout: 200      # 20ms → 200ms（サーバが一瞬詰まっても ACK を拾える）
    bt_loop_duration: 50     # 10ms → 50ms
    # enable_groot_monitoring: False   # tick毎のZMQ送信負荷を削るなら無効化
```

### C. コントローラ側の耐性を上げる（補助）
- `controller_server` の `failure_tolerance` を設定し、一時的な制御失敗で即 abort しないようにする。
- `transform_tolerance` を少し緩める（CPU 遅延で TF が古くなるケースの保険）。

### D. 地図・コストマップ（過去要因への保険）
- SLAM 運用なら余計な `map_server`（`turtlebot3_world.yaml`）を起動しない。
- 必要なら `global_costmap` を `rolling_window: true` 化し、地図端でも範囲外にしない。

---

## 5. まとめ

| 項目 | 内容 |
|------|------|
| **直接原因** | **Jetson の CPU 過負荷**（load 18 / 4コア）。controller が 20Hz を維持できず FollowPath が abort、bt_navigator は 20ms の server_timeout を超過し send_goal failed |
| **最優先対策** | A: RViz をリモートPCで**プロセス起動**（画面転送は不可）＋ Nav2 周期を下げる |
| **即効の緩和** | B: `server_timeout` / `bt_loop_duration` を延長 |

> 設定ファイル: `go2_ros2_ws/install/go2_navigation/share/go2_navigation/config/nav2_params.yaml`
> （実際に反映されるのは **install 側**。src を編集した場合は再ビルド／再配置が必要）
>
> 関連: [planner_server が予期せず停止する問題](nav2_planner_server_troubleshooting.md) /
> [ゴール付近で腰だけ動いて歩かない問題](nav2_goal_wallow_troubleshooting.md)
