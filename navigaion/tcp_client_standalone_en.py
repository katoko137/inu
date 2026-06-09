#!/usr/bin/env python3
import json
import os
import socket
import struct
import time

import yaml


class TcpClient:
    def __init__(self, config_path=None):
        # Use the default config path if none is provided.
        if config_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(
                os.path.dirname(current_dir),
                'config',
                'tcp_config.yaml'
            )

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            self.host = config['nav_server']['host']
            self.port = int(config['nav_server']['port'])
            self.socket = None
            self.connected = False
        except Exception as e:
            print(f'Failed to load config file: {str(e)}')
            # Fall back to the default local connection settings.
            self.host = '127.0.0.1'
            self.port = 5432
            self.socket = None
            self.connected = False

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f'Connected to server {self.host}:{self.port}')
        except Exception as e:
            print(f'Connection failed: {str(e)}')
            print(f'host: {self.host}')
            print(f'port: {self.port}')
            self.connected = False

    def send_goal(self, goal_data):
        if not self.connected:
            print('Not connected to server')
            return False

        try:
            # Convert the goal data to JSON and prefix it with its byte length.
            data = json.dumps(goal_data).encode()
            length = struct.pack('!I', len(data))

            # Send the message length followed by the JSON payload.
            self.socket.sendall(length + data)
            return True

        except Exception as e:
            print(f'Failed to send goal data: {str(e)}')
            return False

    def close(self):
        if self.socket:
            self.socket.close()
            self.connected = False
            print('Connection closed')


def main():
    # Create a TCP client. Pass a custom config path here if needed.
    # client = TcpClient('/path/to/your/config.yaml')
    client = TcpClient()

    # Connect to the TCP server.
    client.connect()

    if not client.connected:
        return

    try:
        while True:
            # Example navigation goal.
            goal_data = {
                'position': {
                    'x': -6.0,
                    'y': -6.0,
                    'z': 0.0
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0
                }
            }

            if client.send_goal(goal_data):
                print('Navigation goal sent')

            time.sleep(1)

    except KeyboardInterrupt:
        print('\nProgram stopped')
    finally:
        client.close()


if __name__ == '__main__':
    main()
