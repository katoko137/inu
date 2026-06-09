#!/usr/bin/env python3
import json
import os
import socket
import struct
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2TcpBridge(Node):
    def __init__(self):
        super().__init__('nav2_tcp_bridge')

        # Load TCP server configuration.
        config_path = os.path.join(
            get_package_share_directory('go2_tcp_toolbox'),
            'config',
            'tcp_config.yaml'
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.host = config['nav_server']['host']
        self.port = int(config['nav_server']['port'])

        # Retry state for navigation goals.
        self.max_retries = 3
        self.current_goal = None
        self.retry_count = 0

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Start the TCP server.
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)

        self.tcp_thread = threading.Thread(target=self.tcp_server_loop)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()

        self.get_logger().info(
            f'Nav2 TCP bridge started. Listening on {self.host}:{self.port}'
        )

    def tcp_server_loop(self):
        while rclpy.ok():
            self.get_logger().info('Waiting for a TCP client connection...')
            client_socket, addr = self.server_socket.accept()
            self.get_logger().info(f'TCP client connected: {addr}')

            try:
                while True:
                    # Read the 4-byte message length.
                    length_data = client_socket.recv(4)
                    if not length_data:
                        break

                    length = struct.unpack('!I', length_data)[0]

                    # Read the JSON payload.
                    data = b''
                    while len(data) < length:
                        chunk = client_socket.recv(length - len(data))
                        if not chunk:
                            break
                        data += chunk

                    if not data:
                        break

                    # Parse the received goal JSON.
                    goal_data = json.loads(data.decode())

                    # Build a Nav2 NavigateToPose goal.
                    goal_msg = NavigateToPose.Goal()
                    goal_msg.pose.header.frame_id = "odom"
                    goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                    goal_msg.pose.pose.position.x = goal_data['position']['x']
                    goal_msg.pose.pose.position.y = goal_data['position']['y']
                    goal_msg.pose.pose.position.z = goal_data['position']['z']
                    goal_msg.pose.pose.orientation.x = goal_data['orientation']['x']
                    goal_msg.pose.pose.orientation.y = goal_data['orientation']['y']
                    goal_msg.pose.pose.orientation.z = goal_data['orientation']['z']
                    goal_msg.pose.pose.orientation.w = goal_data['orientation']['w']

                    # Reset retry tracking for the new goal.
                    self.retry_count = 0
                    self.current_goal = goal_msg

                    self.get_logger().info(
                        f"Received navigation goal: "
                        f"x={goal_msg.pose.pose.position.x}, "
                        f"y={goal_msg.pose.pose.position.y}"
                    )

                    # Wait for the NavigateToPose action server.
                    if not self._action_client.wait_for_server(timeout_sec=5.0):
                        self.get_logger().error('NavigateToPose action server is not available')
                        continue

                    # Send the navigation goal.
                    self.send_navigation_goal(goal_msg)

            except Exception as e:
                self.get_logger().error(f'TCP communication error: {str(e)}')
            finally:
                client_socket.close()
                self.get_logger().info('TCP client connection closed')

    def send_navigation_goal(self, goal_msg):
        """Send a navigation goal to Nav2."""
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal was rejected')
            return

        self.get_logger().info('Navigation goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation in progress. Distance remaining: '
            f'{feedback.distance_remaining:.2f} m'
        )

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded. Goal reached.')
            self.retry_count = 0
        else:
            self.get_logger().info(f'Navigation failed. Status code: {status}')
            if self.retry_count < self.max_retries and self.current_goal is not None:
                self.retry_count += 1
                self.get_logger().info(
                    f'Retrying navigation goal '
                    f'({self.retry_count}/{self.max_retries})'
                )
                self.send_navigation_goal(self.current_goal)
            else:
                self.get_logger().error(
                    f'Navigation failed after {self.max_retries} retries'
                )
                self.retry_count = 0
                self.current_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2TcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
