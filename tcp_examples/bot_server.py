#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('bot_server_node')

        # Set up TCP server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('127.0.0.1', 2000))  # Bind to localhost and port 2000
        self.server_socket.listen(1)
        self.get_logger().info("TCP server listening on port 2000")

        # Create a publisher for controlling the turtle
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Accept connections from the client
        self.client_socket, self.client_address = self.server_socket.accept()
        self.get_logger().info(f"Connection from {self.client_address}")

        # Start listening to commands from the client
        self.listen_to_client()

    def listen_to_client(self):
        try:
            while True:
                # Receive data from the client
                data = self.client_socket.recv(1024)
                if not data:
                    break  # Exit if no data is received
                command = data.decode()
                self.get_logger().info(f"Received command: {command}")

                # Control the turtle based on the command
                self.control_turtle(command)

        except Exception as e:
            self.get_logger().error(f"Error while receiving data: {e}")
        finally:
            self.client_socket.close()

    def control_turtle(self, command: str):
        twist_msg = Twist()

        if command == "move_forward":
            twist_msg.linear.x = 2.0  # Move forward at speed 2
            twist_msg.angular.z = 0.0  # No rotation
        elif command == "move_backward":
            twist_msg.linear.x = -2.0  # Move backward at speed -2
            twist_msg.angular.z = 0.0  # No rotation
        elif command == "turn_left":
            twist_msg.linear.x = 0.0  # No forward movement
            twist_msg.angular.z = 1.0  # Turn left
        elif command == "turn_right":
            twist_msg.linear.x = 0.0  # No forward movement
            twist_msg.angular.z = -1.0  # Turn right
        elif command == "stop":
            twist_msg.linear.x = 0.0  # Stop moving
            twist_msg.angular.z = 0.0  # Stop rotating

        # Publish the twist message to the turtle
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    server = TcpServerNode()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
