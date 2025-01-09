#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses  # To handle real-time keypresses

class TcpClientNode(Node):
    def __init__(self):
        super().__init__('bot_client_node')

        # Create a TCP client socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 2000))  # Connect to the server at localhost:2000

        # Create a publisher to send messages to a ROS topic (optional)
        self.pub = self.create_publisher(String, 'teleop_status', 10)

        self.get_logger().info("TCP Client Node initialized. Use keys for control: ")
        self.get_logger().info("'i' = Move Forward, 'j' = Turn Left, 'm' = Move Backward, 'l' = Turn Right, 'k' = Stop")

        self.teleoperate()

    def send_command(self, command: str):
        """Send the command to the server"""
        self.sock.sendall(command.encode())

        # Optionally, you can log the sent command to a topic
        msg = String()
        msg.data = f"Sent command: {command}"
        self.pub.publish(msg)

    def teleoperate(self):
        """Handle key-based user input and control the TurtleBot"""
        stdscr = curses.initscr()  # Initialize the curses screen
        curses.cbreak()  # Disable line buffering
        stdscr.keypad(1)  # Enable special key input (e.g., arrows)

        try:
            while True:
                key = stdscr.getch()  # Get the pressed key

                if key == ord('i'):  # Move forward
                    self.send_command("move_forward")
                elif key == ord('j'):  # Turn left
                    self.send_command("turn_left")
                elif key == ord('m'):  # Move backward
                    self.send_command("move_backward")
                elif key == ord('l'):  # Turn right
                    self.send_command("turn_right")
                elif key == ord('k'):  # Stop the robot
                    self.send_command("stop")
                elif key == ord('q'):  # Quit (exit) the program
                    self.get_logger().info("Exiting control loop.")
                    break

        finally:
            curses.endwin()  # Clean up curses

        # Close the connection after quitting
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    client = TcpClientNode()

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
