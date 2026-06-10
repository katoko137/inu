#!/usr/bin/env python3
import math
from typing import Dict, List

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


# 使用する地図に合わせて、A/B/C/D地点の座標を編集します。
# yawの単位はラジアンです。例: 0.0 = 正面、1.57 = 左90度。
WAYPOINTS = [
    {"name": "A", "x": -6.0, "y": -6.0, "yaw": 0.0},
    {"name": "B", "x": -4.0, "y": -6.0, "yaw": 0.0},
    {"name": "C", "x": -4.0, "y": -4.0, "yaw": 1.57},
    {"name": "D", "x": -6.0, "y": -4.0, "yaw": 3.14},
]

# Nav2が報告する残り経路距離がこの値以下になったら、次の目的地を送信します。
SWITCH_DISTANCE_M = 0.5

# RVizのmap上に固定した地点へ向かわせる場合は "map" を使います。
# オドメトリ座標系の目的地として扱いたい場合だけ "odom" に変更します。
GOAL_FRAME_ID = "map"

# 最後にAをもう一度追加し、A -> B -> C -> D -> A の巡回ルートにします。
RETURN_TO_START = True

# Trueにすると、最後の地点で終了せず A -> B -> C -> D -> A -> ... と巡回し続けます。
# Trueの場合、RETURN_TO_STARTは使わず、WAYPOINTSをそのまま繰り返します。
LOOP_FOREVER = False

# LOOP_FOREVERがFalseのとき、WAYPOINTSを何回繰り返すかを指定します。
# 例: PATROL_LAPS = 3, RETURN_TO_START = False の場合、A -> B -> C -> D を3回実行します。
PATROL_LAPS = 1

# ノード起動後、最初の目的地を送るまでの待ち時間です。
START_DELAY_SEC = 3.0

# Nav2が失敗を返した場合に、現在の目的地を再送する最大回数です。
MAX_RETRIES = 3


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')

        if len(WAYPOINTS) < 2:
            raise ValueError('At least two waypoints are required for patrol')

        self.loop_forever = LOOP_FOREVER
        self.patrol_laps = PATROL_LAPS
        if not self.loop_forever and self.patrol_laps < 1:
            raise ValueError('PATROL_LAPS must be 1 or greater')

        self.route = self.build_route(
            WAYPOINTS,
            RETURN_TO_START,
            self.loop_forever,
            self.patrol_laps
        )
        self.switch_distance = SWITCH_DISTANCE_M
        self.max_retries = MAX_RETRIES
        self.retry_counts: Dict[int, int] = {}

        self.active_goal_index = None
        self.active_goal_generation = None
        self.goal_generation = 0
        self.completed_laps = 0
        self.current_goal_handle = None
        self.handoff_sent_for_goal = None
        self.intentional_cancel_generations = set()
        self.route_finished = False
        self.started = False

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.start_timer = self.create_timer(START_DELAY_SEC, self.start_patrol)

        self.get_logger().info('巡回ノードを起動しました')
        self.get_logger().info(f'目的地の座標系: {GOAL_FRAME_ID}')
        self.get_logger().info(f'次の目的地へ切り替える残り距離: {self.switch_distance:.2f} m')
        self.get_logger().info(f'無限巡回モード: {"有効" if self.loop_forever else "無効"}')
        self.get_logger().info(
            f'指定周回数: {"無制限" if self.loop_forever else str(self.patrol_laps) + "周"}'
        )
        self.get_logger().info('巡回ルート: ' + ' -> '.join(point['name'] for point in self.route))

    def cancel_current_goal(self, reason='', after_cancel=None):
        if self.current_goal_handle is None:
            self.get_logger().info('キャンセル対象のNav2 Goalはありません')
            if after_cancel is not None and not self.route_finished:
                after_cancel()
            return

        reason_text = f' 理由={reason}' if reason else ''
        self.get_logger().info(f'現在のNav2 Goalをキャンセルします。{reason_text}')
        if self.active_goal_generation is not None:
            self.intentional_cancel_generations.add(self.active_goal_generation)
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda future, callback=after_cancel:
            self.cancel_done_callback(future, callback)
        )
        self.current_goal_handle = None

    def cancel_done_callback(self, future, after_cancel=None):
        try:
            cancel_response = future.result()
        except Exception as e:
            self.get_logger().error(f'Nav2 Goalキャンセル結果の取得に失敗しました: {str(e)}')
        else:
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Nav2 Goalのキャンセル要求が受理されました')
            else:
                self.get_logger().info('キャンセル対象のNav2 Goalはありませんでした')

        if after_cancel is not None and not self.route_finished:
            after_cancel()

    @staticmethod
    def build_route(
        waypoints: List[Dict[str, float]],
        return_to_start: bool,
        loop_forever: bool,
        patrol_laps: int
    ):
        if loop_forever:
            return list(waypoints)

        route = []
        for _ in range(patrol_laps):
            route.extend(dict(waypoint) for waypoint in waypoints)

        if return_to_start:
            start = dict(waypoints[0])
            start['name'] = f"{start['name']}_return"
            route.append(start)
        return route

    def start_patrol(self):
        """起動後に最初の目的地を送信します。"""
        if self.started:
            return

        self.started = True
        self.start_timer.cancel()

        self.get_logger().info('Nav2のNavigateToPose action serverを待機しています...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2のNavigateToPose action serverが見つかりません')
            self.route_finished = True
            return

        self.send_goal_by_index(0, reason='巡回開始')

    def make_goal(self, waypoint):
        """地点情報の辞書をNav2のNavigateToPose Goalへ変換します。"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = GOAL_FRAME_ID
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(waypoint['x'])
        goal_msg.pose.pose.position.y = float(waypoint['y'])
        goal_msg.pose.pose.position.z = 0.0

        yaw = float(waypoint['yaw'])
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal_msg

    def send_goal_by_index(self, goal_index, reason=''):
        """指定した番号の目的地をNav2へ送信します。"""
        if self.route_finished:
            return

        if goal_index >= len(self.route):
            if self.loop_forever:
                goal_index = 0
            else:
                self.finish_route()
                return

        waypoint = self.route[goal_index]
        goal_msg = self.make_goal(waypoint)
        self.goal_generation += 1
        goal_generation = self.goal_generation
        self.active_goal_index = goal_index
        self.active_goal_generation = goal_generation

        reason_text = f' ({reason})' if reason else ''
        self.get_logger().info(
            f"Nav2へ目的地を送信します: "
            f"{goal_index + 1}/{len(self.route)}番目={waypoint['name']} "
            f"x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}"
            f"{reason_text} / goal_id={goal_generation}"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, index=goal_index, generation=goal_generation:
            self.feedback_callback(feedback_msg, index, generation)
        )
        send_goal_future.add_done_callback(
            lambda future, index=goal_index, generation=goal_generation:
            self.goal_response_callback(future, index, generation)
        )

    def goal_response_callback(self, future, goal_index, goal_generation):
        """送信した目的地に対するNav2の受理/拒否応答を処理します。"""
        if (
            goal_index != self.active_goal_index or
            goal_generation != self.active_goal_generation
        ):
            self.get_logger().debug(
                f'古い目的地応答を無視しました: index={goal_index}, goal_id={goal_generation}'
            )
            return

        waypoint = self.route[goal_index]
        self.get_logger().info(
            f"Nav2から目的地応答を受信しました: 目的地={waypoint['name']}"
        )

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Nav2が目的地を拒否しました: {waypoint['name']}"
            )
            self.retry_or_finish(goal_index)
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(
            f"Nav2が目的地を受理しました: {waypoint['name']}。移動結果を待ちます"
        )
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda result_future, index=goal_index, generation=goal_generation:
            self.get_result_callback(result_future, index, generation)
        )

    def feedback_callback(self, feedback_msg, goal_index, goal_generation):
        """残り距離を監視し、目的地付近に入ったら次の目的地を送信します。"""
        if self.route_finished:
            return

        if (
            goal_index != self.active_goal_index or
            goal_generation != self.active_goal_generation
        ):
            return

        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        waypoint = self.route[goal_index]

        self.get_logger().info(
            f"Nav2から移動中フィードバックを受信: "
            f"現在の目的地={waypoint['name']} / "
            f"目的地までの残り距離={distance:.2f} m / "
            f"切り替え閾値={self.switch_distance:.2f} m"
        )

        handoff_key = (goal_index, goal_generation)
        if handoff_key == self.handoff_sent_for_goal:
            return

        if distance > self.switch_distance:
            return

        self.handoff_sent_for_goal = handoff_key
        next_index = goal_index + 1

        if next_index >= len(self.route):
            if self.loop_forever:
                self.completed_laps += 1
                next_index = 0
                next_waypoint = self.route[next_index]
                self.get_logger().info(
                    f"閾値到達: {waypoint['name']}までの残り距離が"
                    f"{distance:.2f} mになりました。"
                    f"{self.completed_laps}周目が完了したため、"
                    f"現在のGoalをキャンセルしてから、"
                    f"次の目的地={next_waypoint['name']}へ戻って巡回を継続します"
                )
                self.cancel_current_goal(
                    reason='次の目的地へ切り替えるため',
                    after_cancel=lambda index=next_index:
                    self.send_goal_by_index(index, reason='無限巡回で先頭へ戻る')
                )
                return

            self.get_logger().info(
                f"閾値到達: 最終目的地={waypoint['name']}までの残り距離が"
                f"{distance:.2f} mになりました。巡回を完了します"
            )
            self.finish_route()
            return

        next_waypoint = self.route[next_index]
        self.get_logger().info(
            f"閾値到達: {waypoint['name']}までの残り距離が"
            f"{distance:.2f} mになりました。"
            f"現在のGoalをキャンセルしてから、"
            f"次の目的地={next_waypoint['name']}を送信します"
        )
        self.cancel_current_goal(
            reason='次の目的地へ切り替えるため',
            after_cancel=lambda index=next_index:
            self.send_goal_by_index(index, reason='残り距離が閾値以下')
        )

    def get_result_callback(self, future, goal_index, goal_generation):
        """目的地への移動が完了したときの最終結果を処理します。"""
        if self.route_finished:
            self.get_logger().debug(
                f'巡回完了後に届いたNav2結果を無視しました: index={goal_index}, goal_id={goal_generation}'
            )
            return

        status = future.result().status
        waypoint = self.route[goal_index]
        status_text = self.status_to_text(status)

        if goal_generation in self.intentional_cancel_generations:
            self.intentional_cancel_generations.discard(goal_generation)
            self.get_logger().info(
                f"次の目的地へ切り替えるためにキャンセルしたNav2結果を無視します: "
                f"目的地={waypoint['name']} / 結果={status_text}({status}) / "
                f"goal_id={goal_generation}"
            )
            return

        if (
            goal_index != self.active_goal_index or
            goal_generation != self.active_goal_generation
        ):
            self.get_logger().debug(
                f'古い目的地結果を無視しました: index={goal_index}, goal_id={goal_generation}'
            )
            return

        self.get_logger().info(
            f"Nav2から最終結果を受信: 目的地={waypoint['name']} / "
            f"結果={status_text}({status})"
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Nav2の到着結果を受信しました: {waypoint['name']}。"
                f"ただし巡回の切り替え/完了判定は残り距離の閾値で行います"
            )
            self.retry_counts[goal_index] = 0
            return

        self.get_logger().warning(
            f"目的地への移動が成功しませんでした: {waypoint['name']} / "
            f"結果={status_text}({status})"
        )
        self.retry_or_finish(goal_index)

    def retry_or_finish(self, goal_index):
        retries = self.retry_counts.get(goal_index, 0)
        if retries < self.max_retries:
            self.retry_counts[goal_index] = retries + 1
            self.get_logger().info(
                f"目的地を再送信します: {self.route[goal_index]['name']} "
                f"({self.retry_counts[goal_index]}/{self.max_retries})"
            )
            self.send_goal_by_index(goal_index, reason='再試行')
            return

        self.get_logger().error(
            f"再試行上限に達したため巡回を停止します: "
            f"{self.route[goal_index]['name']}"
        )
        self.finish_route()

    def finish_route(self):
        if self.route_finished:
            return

        self.route_finished = True
        self.cancel_current_goal(reason='巡回完了')
        self.get_logger().info('巡回処理が完了しました')

    @staticmethod
    def status_to_text(status):
        status_names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return status_names.get(status, 'UNRECOGNIZED')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('巡回処理をキーボード入力で中断しました')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
