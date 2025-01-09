#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
import socket
import struct
from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import Temperature
from std_srvs.srv import SetBool  # Corrected import for SetBool service



class SensorServer(Node):
    def __init__(self):
        super().__init__('sensor_server')

        # Declare parameter for the interval (in milliseconds)
        self.declare_parameter('interval', 1000)  # Default interval 1000 ms
        self.interval = self.get_parameter('interval').get_parameter_value().integer_value

        # Create publishers for various sensor data
        self.voltage_pub = self.create_publisher(String, 'sensor/supply_voltage', 10)
        self.temp_pub = self.create_publisher(Temperature, 'sensor/env_temp', 10)
        self.yaw_pub = self.create_publisher(String, 'sensor/yaw', 10)
        self.pitch_pub = self.create_publisher(String, 'sensor/pitch', 10)
        self.roll_pub = self.create_publisher(String, 'sensor/roll', 10)

        # Setup the TCP server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 2000))
        self.server_socket.listen(1)
        self.get_logger().info("Sensor Server started, waiting for client...")

        # Wait for a connection from the client
        self.client_socket, self.client_address = self.server_socket.accept()
        self.get_logger().info(f"Client connected: {self.client_address}")

        # Create a callback group for reentrant behavior (allows service calls and timer to run concurrently)
        self.callback_group = ReentrantCallbackGroup()

        # Timer for periodic sensor status updates
        self.timer = self.create_timer(self.interval / 1000.0, self.send_status, callback_group=self.callback_group)

        # Create services for starting and stopping the sensor
        self.create_service(SetBool, 'start_sensor', self.start_sensor)
        self.create_service(SetBool, 'stop_sensor', self.stop_sensor)

    def send_status(self):
        # Simulated sensor data
        supply_voltage = 9800  # in milli-volts
        env_temp = 100  # in deci-Celsius
        yaw = 7  # in deci-degrees
        pitch = 9  # in deci-degrees
        roll = 3  # in deci-degrees

        # Publish the data to ROS topics
        self.publish_data(supply_voltage, 'supply_voltage')
        self.publish_data(env_temp, 'env_temp')
        self.publish_data(yaw, 'yaw')
        self.publish_data(pitch, 'pitch')
        self.publish_data(roll, 'roll')

        # Send status message back to the client
        status_message = self.create_status_message(supply_voltage, env_temp, yaw, pitch, roll)
        self.client_socket.sendall(status_message)

    def publish_data(self, value, topic_name):
        if topic_name == 'supply_voltage':
            message = String()
            message.data = f"{value}mV"
            self.voltage_pub.publish(message)
        elif topic_name == 'env_temp':
            message = Temperature()
            message.temperature = value / 10.0  # Convert to Celsius
            self.temp_pub.publish(message)
        elif topic_name == 'yaw':
            message = String()
            message.data = f"{value / 10.0} degrees"
            self.yaw_pub.publish(message)
        elif topic_name == 'pitch':
            message = String()
            message.data = f"{value / 10.0} degrees"
            self.pitch_pub.publish(message)
        elif topic_name == 'roll':
            message = String()
            message.data = f"{value / 10.0} degrees"
            self.roll_pub.publish(message)

    def create_status_message(self, supply_voltage, env_temp, yaw, pitch, roll):
        # Encode the status message in hexadecimal format
        message = "Start11"
        message += self.encode_value(supply_voltage)
        message += self.encode_value(env_temp)
        message += self.encode_value(yaw)
        message += self.encode_value(pitch)
        message += self.encode_value(roll)
        message += "End\r\n"
        return message.encode()

    def encode_value(self, value):
        # Little endian encoding of 16-bit integer to hex string
        return format(value, '04x').upper()

    def start_sensor(self, request, response):
        # Start the sensor by enabling the timer
        self.get_logger().info("Starting sensor...")
        self.timer = self.create_timer(self.interval / 1000.0, self.send_status, callback_group=self.callback_group)
        response.success = True
        response.message = "Sensor started."
        return response

    def stop_sensor(self, request, response):
        # Stop the sensor by disabling the timer
        self.get_logger().info("Stopping sensor...")
        self.timer.cancel()
        response.success = True
        response.message = "Sensor stopped."
        return response

    def stop(self):
        self.client_socket.close()
        self.get_logger().info("Client connection closed.")
        self.server_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = SensorServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
