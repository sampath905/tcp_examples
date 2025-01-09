#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct

class SensorClient(Node):
    def __init__(self):
        super().__init__('sensor_client')

        # Get parameter for the interval (in milliseconds)
        self.declare_parameter('interval', 1000)  # Default interval 1000 ms
        self.interval = self.get_parameter('interval').get_parameter_value().integer_value

        # Create a TCP connection to the server
        self.server_ip = '127.0.0.1'
        self.server_port = 2000
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.connect((self.server_ip, self.server_port))

        # Send Start command
        self.send_start_command()

    def send_start_command(self):
        # Command ID for Start Message = 03, Interval (in milliseconds)
        command = "Start03"
        payload = struct.pack('<H', self.interval)  # Little-endian encoding of interval
        command_message = command + payload.hex().upper() + "End\r\n"
        self.server_socket.sendall(command_message.encode())
        self.get_logger().info("Sent Start Command")

    def receive_status(self):
        # Listen for status message from the server
        data = self.server_socket.recv(1024)
        self.decode_and_publish_status(data)

    def decode_and_publish_status(self, data):
        # Decode the status message and publish the values (simulated)
        self.get_logger().info(f"Received Data: {data.decode()}")

    def stop(self):
        # Send Stop command
        command = "Start09End\r\n"
        self.server_socket.sendall(command.encode())
        self.get_logger().info("Sent Stop Command")
        self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = SensorClient()

    try:
        while rclpy.ok():
            node.receive_status()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
