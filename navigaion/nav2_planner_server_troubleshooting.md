# Nav2 `planner_server` が予期せず停止する問題

作成日: 2026-06-18
対象環境: Unitree Go2 内部 Jetson / ROS2 Foxy / Nav2（RViz + Nav2 で移動制御）

---

## 1. 問題（Problem）

Go2 をナビゲーションさせている最中に、以下のメッセージが表示され Nav2 スタックが停止する。

```
planner_server が予期せず停止しました
```

このメッセージは **`lifecycle_manager`** が出力するもの。Nav2 の各サーバー
（planner_server / controller_server など）は `lifecycle_manager` と **bond（心拍監視）**
で接続されており、サーバーからの心拍が **`bond_timeout`（既定 4 秒）以内に届かない**と、
`lifecycle_manager` が「予期せず停止」と判断してスタック全体を落とす。

> launch は標準の `nav2_bringup/navigation_launch.py` を使用
> （`go2_navigation/launch/go2_nav2.launch.py`）。
> `bond_timeout` / `respawn` は **既定のまま（4 秒・自動復帰なし）**。

---

## 2. 現象（Phenomenon）

`planner_server` のログ（`~/.ros/log/planner_server_*.log`）が、以下の警告で
埋め尽くされていた。

```
[WARN] [nav2_costmap_2d]: Robot is out of bounds of the costmap!
[WARN] [global_costmap.global_costmap]: Sensor origin at (0.52, 0.71) is out of map bounds.
       The costmap cannot raytrace for it.
```

= **ロボットの位置がグローバルコストマップの範囲外**になっている。

### 確認したログ（根拠）

**ファイル:** `~/.ros/log/planner_server_14342_1781775479534.log`
（`~/.ros/log/planner_server_<PID>_<timestamp>.log` の形式。PID・時刻は実行ごとに変わる）

**`tail -40` で表示された内容:** 約 1 秒周期（コストマップ update_frequency=1.0Hz に一致）で
下記 2 行が延々と繰り返されていた。エラー（ERROR）ではなく WARN だが、止まらず出続けて
いた点が異常。

```
[WARN] [1781775487.780145856] [nav2_costmap_2d]: Robot is out of bounds of the costmap!
[WARN] [1781775487.780250948] [global_costmap.global_costmap]: Sensor origin at (0.52, 0.71) is out of map bounds. The costmap cannot raytrace for it.
[WARN] [1781775488.780158175] [nav2_costmap_2d]: Robot is out of bounds of the costmap!
[WARN] [1781775488.780247170] [global_costmap.global_costmap]: Sensor origin at (0.52, 0.71) is out of map bounds. The costmap cannot raytrace for it.
   …（以降、同じ 2 行が秒単位で継続）…
```

**読み取れたこと:**
- `Sensor origin at (0.52, 0.71)` … センサ（LiDAR）原点が地図のごく狭い範囲（0.5m 付近）に
  しか収まっておらず、地図そのものが極端に小さい／原点がずれていることを示す。
- WARN が秒単位で出続ける＝コストマップが毎周期ロボットを覆えず raytrace 失敗している。

**ログディレクトリ確認コマンド（実行したもの）:**
```bash
ls -lt ~/.ros/log/ | head -8                              # 最新ログ一覧
tail -40 ~/.ros/log/planner_server_14342_1781775479534.log # 上記の内容を確認
```

> なお、同ディレクトリの他ノードのログ（`bt_navigator_*.log`,
> `lifecycle_manager_*.log`, `async_slam_toolbox_node_*.log` など）は
> サイズ 0 byte で、内容は planner_server ログにのみ記録されていた。

### 原因の連鎖
```
グローバルコストマップがロボット位置を覆っていない
   ↓
NavFn プランナが経路を作れず失敗を繰り返す ＋ Jetson に負荷
   ↓
planner_server が停滞 → 4 秒以内に心拍を返せない
   ↓
lifecycle_manager「planner_server が予期せず停止」→ スタック停止
```

### なぜ範囲外になるのか
`global_costmap` は `rolling_window` ではなく **`static_layer`** を使用
（`config/nav2_params.yaml`）。
→ コストマップのサイズ＝購読している `/map` のサイズ。

以下のいずれかでロボットが地図範囲外になる:
- SLAM（async_slam_toolbox）の地図がまだ小さい / 原点がずれている
- TF `map → base_link` が地図外を指している
- 余計な `map_server` が小さなシム地図（`turtlebot3_world.yaml`）を `/map` に流し、
  `static_layer`（`map_subscribe_transient_local: True`）がそちらを掴んでいる

> 注: `navigation_launch.py` は `map_server` を起動しないため、
> 設定内の `yaml_filename: "turtlebot3_world.yaml"` は通常は未使用。
> ただし別プロセスで map_server が起動していると上記の競合が起きる。

### 確認した設定・launch ファイル（根拠）

**設定ファイル:** `~/go2_ros2_ws/install/go2_navigation/share/go2_navigation/config/nav2_params.yaml`
（実際に反映される install 側）

`grep` で確認した該当箇所:
```yaml
# global_costmap が static_layer ベース（rolling_window ではない）
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]   # 215 行付近
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True                            # 226-228 行付近

# map_server が小さなシム地図を指している
map_server:
  ros__parameters:
    yaml_filename: "turtlebot3_world.yaml"                             # 244 行

# planner の更新頻度が高い（負荷要因）
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0                                   # 256 行
```

なお `local_costmap` 側は `rolling_window: true` / `width: 3` / `height: 3`
（168-170 行付近）で、グローバルとローカルで方式が異なることも確認した。

**launch ファイル:** `~/go2_ros2_ws/install/go2_navigation/share/go2_navigation/launch/go2_nav2.launch.py`

標準の `nav2_bringup/navigation_launch.py` を `IncludeLaunchDescription` で
取り込んでいるだけ（24 行）。`bond_timeout` や `respawn` の明示指定は無く、
`grep` でも該当ヒット無し＝**すべて nav2 既定値**であることを確認した。

---

## 3. 解決方法（Solution）

### A. 起きた直後の復旧
1. **Nav2 スタックを再起動**（再 launch）。
   `lifecycle_manager` は `autostart=True` のため起動し直せば復帰する。
2. 復帰後、RViz で **地図が表示され、ロボットがその地図内にいるか**を必ず確認。

### B. 根本対処（再発防止）— 重要度順

#### ① コストマップがロボットを覆うようにする（最優先）
稼働中に切り分け:
```bash
ros2 topic echo /map --field info          # 地図のサイズ・原点を確認
ros2 run tf2_ros tf2_echo map base_link    # ロボットが地図範囲内か確認
ros2 node list | grep map_server           # 余計な map_server が居ないか確認
```
- SLAM 運用なら **map_server で `turtlebot3_world.yaml` を読ませない**
  （`/map` の二重発行をなくす）。
- 既存地図でナビするなら `yaml_filename` を実環境の地図（`~/maps/` 内）に変更し、
  初期位置を地図内に合わせる。

#### ② コストマップを必ずロボット追従にする（保険・代替）
`global_costmap` を以下に変更すると、地図端でも範囲外にならず planner 失敗を回避できる
（static_layer 中心の SLAM 運用とはトレードオフ）。
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: true
      width: 10
      height: 10
```

#### ③ Jetson での bond クラッシュ自体を緩和
- `lifecycle_manager` の **`bond_timeout` を延ばす（例: 10.0）か 0 で無効化**
  → Jetson で「ノードが突然死ぬ」定番対策。
- **ノードに respawn を付けて自動復帰**させる
  （Foxy の `navigation_launch.py` は respawn 非対応のため、コピーして改造するか
  自前 launch で起こす）。
- CPU 負荷軽減: `expected_planner_frequency: 20.0` は高すぎるので **1〜5** に。
  NavFn を 20Hz で回す必要はない。

---

## 4. まとめ

| 項目 | 内容 |
|------|------|
| **表面的な事象** | bond タイムアウトによる `planner_server` 停止 |
| **真の引き金** | コストマップ範囲外＝**地図とロボット位置の不整合** |
| **最優先対処** | ① 地図 / TF の確認・修正（`/map` の二重発行解消、初期位置合わせ） |
| **再発防止** | ③ `bond_timeout` 延長 ＋ respawn、負荷軽減 |

> 設定ファイル: `go2_ros2_ws/install/go2_navigation/share/go2_navigation/config/nav2_params.yaml`
> （実際に反映されるのは **install 側**。src 側を編集した場合は再ビルド／再配置が必要）
