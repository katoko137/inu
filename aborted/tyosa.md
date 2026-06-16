以下を実機ターミナルで実行してください。
# 1. bt_navigator のログ
grep -R "bt_navigator" ~/.ros/log/latest
# 2. controller_server のログ
grep -R "controller_server" ~/.ros/log/latest
# 3. planner_server のログ
grep -R "planner_server" ~/.ros/log/latest
# 4. recoveries_server のログ
grep -R "recoveries_server" ~/.ros/log/latest
# 5. TF: map -> odom -> base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
# 6. /cmd_vel が出ているか
ros2 topic echo /cmd_vel
# 7. /plan が生成されているか
ros2 topic echo /plan
ABORTED の原因を絞るなら、追加でこれも便利です。
grep -R "ABORT\|Failed\|failed\|Error\|error\|Exception\|Timed out\|TF\|transform\|FollowPath\|Comp
