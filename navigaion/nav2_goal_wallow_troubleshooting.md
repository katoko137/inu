# Go2 がゴール付近で腰だけ動かして歩かない問題

作成日: 2026-06-18
対象環境: Unitree Go2 内部 Jetson / ROS2 Foxy / Nav2（RViz + Nav2 で移動制御）

---

## 1. 問題（Problem）

Go2 が目的地（ゴール）付近まで来たとき、**腰（胴体）を揺らすだけで脚を踏み出さず、
その場から進まない**ことがある。

---

## 2. 現象とメカニズム（Phenomenon）

Go2 の内蔵モーション制御（sport_mode の Move API）には **最低歩行速度のデッドゾーン**が
ある。速度が小さすぎると「脚を上げて踏み出す」動作にならず、**重心移動＝腰の揺れだけ**に
なる。ゴール付近でこれが発生している。

### 原因の連鎖
```
ゴール接近 → Nav2 が減速し微小速度を出力（min 速度 = 0 のため）
         → go2_base が微小 Move をそのまま転送（==0 でないので Stop しない）
         → Go2 の Move API は最低歩行速度未満 → 踏み出せず重心移動だけ＝腰の揺れ
```

### 根拠（設定とコードの連鎖）

#### ① Nav2/DWB が微小速度の出力を許可している
`config/nav2_params.yaml`（FollowPath / DWBLocalPlanner）
```yaml
min_vel_x: 0.0
min_speed_xy: 0.0
min_speed_theta: 0.0
decel_lim_x: -2.5                 # ゴール手前で減速
RotateToGoal.slowing_factor: 5.0  # 最終姿勢合わせを極端に低速化
```
最低速度が全て `0` なので、ゴール接近で減速した Nav2 は
`vx=0.03`、`vyaw=0.02` のような **ゼロではない極小値**を出し続ける。

#### ② ゴール許容値の二重設定で「中途半端ゾーン」が発生
```yaml
# goal_checker（ナビ完了判定 / SimpleGoalChecker）
xy_goal_tolerance: 0.5    # ※ install 側は 0.40
yaw_goal_tolerance: 3.14  # ← 実質どの向きでも OK

# FollowPath / DWB（ローカル制御）
xy_goal_tolerance: 0.25
```
DWB は 0.25m 以内に入ると `RotateToGoal` で並進をやめ、姿勢合わせの微小回転に切り替える。
一方ナビ完了判定は 0.4〜0.5m。**この中間ゾーンで踏み出せない微小速度を出し続ける**。

#### ③ go2_base が「完全ゼロ」しか停止に変換しない
`go2_core/src/go2_base.cpp`（handleVelocity）
```cpp
unitree_api::msg::Request req;
sport_client_->Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
sport_pub_->publish(req);   // 微小速度でも Move(api_id=1008) をそのまま転送
if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) {
    stopRobot();            // ← 3 軸すべて厳密に 0 のときだけ StopMove
}
```
Nav2 の微小残差（0.03 等）は `== 0` を満たさないため **StopMove が送られず、
踏み出せない Move 指令が流れ続ける** → Go2 が腰を揺らすだけになる。

### 根拠として確認したファイル・ログ

> 本件は **クラッシュログではなく、設定ファイル・ソースコード・稼働中トピックの
> ライブ観測**から原因を特定した（planner_server 停止問題のような専用エラーログは無い）。

**(a) 設定ファイル:**
`~/go2_ros2_ws/install/go2_navigation/share/go2_navigation/config/nav2_params.yaml`（install 側）
`grep` で確認した該当箇所:
```yaml
# DWB が微小速度を許可（最低速度すべて 0）
min_vel_x: 0.0          # 116 行
min_vel_y: 0.0          # 117 行
min_speed_xy: 0.0       # 121 行
min_speed_theta: 0.0    # 123 行
# 二重の xy_goal_tolerance
xy_goal_tolerance: 0.40 # goal_checker  108 行（install 側は 0.40）
yaw_goal_tolerance: 3.14 # 110 行
xy_goal_tolerance: 0.25 # FollowPath/DWB 140 行
RotateToGoal.slowing_factor: 5.0 # 152 行付近
```

**(b) サブスクライバのソース:**
`~/go2_ros2_ws/src/go2_ros2_toolbox/go2_core/src/go2_base.cpp`
→ `handleVelocity()`（84-96 行）で `cmd_vel` を受け、`SportClient::Move()` で変換し
`/api/sport/request` に publish。`if(linear.x==0 && linear.y==0 && angular.z==0)` の
**完全ゼロ判定のときだけ `stopRobot()`** を呼ぶ実装を確認（93 行）。

**(c) Move API のソース:**
`~/go2_ros2_ws/src/go2_ros2_toolbox/unitree_pkgs/go2_sport_api/src/common/ros2_sport_client.cpp`
→ `Move()`（43-51 行）が JSON `{x,y,z}` を作り `api_id = ROBOT_SPORT_API_ID_MOVE` を設定。
ヘッダ `include/common/ros2_sport_client.h:16` で
`const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;` を確認。

**(d) 稼働中トピックのライブ観測（実コマンド）:**
```bash
ros2 topic echo /api/sport/request
```
表示された内容（go2_base が実際に流していた Move 指令）:
```yaml
header:
  identity:
    api_id: 1008                         # = MOVE
  policy:
    noreply: true
parameter: '{"x":0.25138068199157715,"y":-0.03523313254117966,"z":4.669545305979739e-10}'
```
→ `api_id:1008`（MOVE）＋速度 JSON が流れており、(b)(c) のコードパスが実機で
動作していることを裏付け。ゴール付近では、この x/y/z が最低歩行速度未満の
極小値（かつ完全ゼロではない）になることで腰揺れが発生する。

---

## 3. 補足: `xy_goal_tolerance` が 2 つある理由

両者は異なる階層のパラメータで役割が違う。

| 場所 | プラグイン | 役割 |
|------|-----------|------|
| `goal_checker:` | `SimpleGoalChecker` | **「ナビゲーション完了か？」の最終判定**。controller_server がこれを見て成功＝停止する |
| `FollowPath:` | `DWBLocalPlanner` | **DWB 内部の挙動切替**。ゴール圏内で並進をやめ `RotateToGoal`（最終回転）に移るための内部しきい値 |

歴史的に、DWB はプラガブルな goal_checker 導入前から自前のゴール許容値を持っていた名残。
Nav2 でよく混乱の元になる重複ポイント。

### 補足: `min_vel_x` と `min_speed_xy` の違い（対策で混同しないこと）

| パラメータ | 種類 | 意味 |
|-----------|------|------|
| `min_vel_x` | 軸ごとの成分の下限 | x 単独の速度サンプル範囲の下限（0 なら後退しない） |
| `min_speed_xy` | 合成速度の大きさの下限 | √(vx²+vy²) がこれ未満の軌道を「停止扱い＝無効」とする閾値 |

- **デッドゾーン対策に使うべきは `min_speed_xy`**。
- `min_vel_x` を上げると x 速度サンプルから「0」が消え、**正常に減速・停止できなくなる**
  別不具合を招くため不可。

---

## 4. 解決方法（Solution）

### 設定ファイルのみで概ね解消可能

#### ① 最重要: `min_speed_xy` / `min_speed_theta` に Go2 の最低歩行速度を設定
DWB は「並進が `min_speed_xy` 未満 かつ 回転が `min_speed_theta` 未満」の軌道を無効化し、
**停止（完全ゼロ）の軌道だけは別枠で常に許可**する。
```yaml
min_speed_xy: 0.12      # Go2 の最低歩行速度（≈0.1〜0.15 m/s、要実測調整）
min_speed_theta: 0.15
```
→ Nav2 の出力が **「実際に歩ける速度」か「完全ゼロ」の二択**になる。
→ 完全ゼロなら `go2_base.cpp` の `if(==0)` が成立し **StopMove が飛ぶ**。
   go2_base の「厳密ゼロしか停止にしない」制約とちょうど噛み合う。

#### ② ゴール許容値の整合
`goal_checker` と DWB の `xy_goal_tolerance` を揃える、または goal_checker 側を
少し広げて早めに「完了」させ、中間ゾーンを無くす。

#### ③ 最終回転の低速化を緩和
`RotateToGoal.slowing_factor: 5.0` を下げる（微小 yaw を防ぐ）。
yaw 許容は既に 3.14（実質オフ）なので影響は小。

### より堅牢にしたい場合（コード対応）
`go2_base.cpp` に **速度デッドバンド**を入れる:
```cpp
const float th = 0.1;  // Go2 の最低歩行速度
if (fabs(vx) < th && fabs(vy) < th && fabs(vyaw) < th) {
    stopRobot();       // 微小速度はまとめて停止に変換
    return;
}
```

### 注意（設定だけでは残るリスク）
- Go2 の最低歩行速度の**実測値が不明**だと `min_speed_xy` のチューニングが試行錯誤になる。
  低すぎれば再発、高すぎればゴール手前で早めに止まる（精度低下）。
- DWB の軌道生成・実機の速度追従誤差で、ごく稀に微小残差が漏れる可能性はゼロではない。

---

## 5. まとめ

| 項目 | 内容 |
|------|------|
| **現象** | ゴール付近で腰だけ揺れて歩かない |
| **真因** | Nav2 がゴール手前で出す**微小速度**が Go2 の最低歩行速度未満。かつ go2_base が完全ゼロしか停止に変換しない |
| **最優先対処** | ① `min_speed_xy` / `min_speed_theta` に最低歩行速度を設定し、出力を「実歩行速度 or 完全ゼロ」に二極化 |
| **堅牢化** | go2_base.cpp に速度デッドバンドを追加 |

> 設定ファイル: `go2_ros2_ws/install/go2_navigation/share/go2_navigation/config/nav2_params.yaml`
> （実際に反映されるのは **install 側**。src 側を編集した場合は再ビルド／再配置が必要）
>
> 関連: [Nav2 planner_server が予期せず停止する問題](nav2_planner_server_troubleshooting.md)
