#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.parameter import Parameter
import curses
import time

class SensorControlClient(Node):
    def __init__(self):
        super().__init__('sensor_control_client')

        # Declare the 'interval' parameter with a default value (e.g., 1000 ms)
        self.declare_parameter('interval', 1000)

        # Create service clients for controlling the sensor
        self.start_client = self.create_client(SetBool, '/start_sensor')
        self.stop_client = self.create_client(SetBool, '/stop_sensor')

        # Wait for the services to become available
        while not self.start_client.wait_for_service(timeout_sec=1.0) or not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /start_sensor or /stop_sensor services to become available...')

    def call_service(self, client, start: bool):
        request = SetBool.Request()
        request.data = start
        client.call_async(request)

    def set_interval(self, interval: int):
        # Set the parameter value after declaring it
        self.set_parameters([Parameter('interval', Parameter.Type.INTEGER, interval)])
        self.get_logger().info(f"Interval set to {interval} ms")


def main(args=None):
    rclpy.init(args=args)
    node = SensorControlClient()

    # Initialize curses screen
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.refresh()

    # Display instructions for the user
    stdscr.addstr(0, 0, "Welcome to the Sensor Control Client")
    stdscr.addstr(1, 0, "Press '1' to start the sensor")
    stdscr.addstr(2, 0, "Press '2' to stop the sensor")
    stdscr.addstr(3, 0, "Press '3' to change the interval")
    stdscr.addstr(4, 0, "Press 'q' to quit the application.")
    stdscr.refresh()

    try:
        while rclpy.ok():
            key = stdscr.getch()  # Get the pressed key

            if key == 49:  # Numpad 1
                stdscr.clear()  # Clear the screen
                stdscr.addstr(0, 0, "Starting the sensor...")
                stdscr.refresh()
                node.call_service(node.start_client, True)  # Start the sensor
                stdscr.addstr(2, 0, "Sensor started.")
                stdscr.refresh()
                time.sleep(1)
                stdscr.clear()
                stdscr.refresh()
                stdscr.addstr(0, 0, "Press '1' to start the sensor")
                stdscr.addstr(1, 0, "Press '2' to stop the sensor")
                stdscr.addstr(2, 0, "Press '3' to change the interval")
                stdscr.addstr(3, 0, "Press 'q' to quit the application.")
                stdscr.refresh()

            elif key == 50:  # Numpad 2
                stdscr.clear()
                stdscr.addstr(0, 0, "Stopping the sensor...")
                stdscr.refresh()
                node.call_service(node.stop_client, False)  # Stop the sensor
                stdscr.addstr(2, 0, "Sensor stopped.")
                stdscr.refresh()
                time.sleep(1)
                stdscr.clear()
                stdscr.refresh()
                stdscr.addstr(0, 0, "Press '1' to start the sensor")
                stdscr.addstr(1, 0, "Press '2' to stop the sensor")
                stdscr.addstr(2, 0, "Press '3' to change the interval")
                stdscr.addstr(3, 0, "Press 'q' to quit the application.")
                stdscr.refresh()

            elif key == 51:  # Numpad 3
                stdscr.clear()
                stdscr.addstr(0, 0, "Enter new interval value (ms): ")
                stdscr.refresh()
                interval = int(stdscr.getstr().decode('utf-8'))  # Input from user
                node.set_interval(interval)  # Set the interval parameter
                stdscr.addstr(2, 0, f"Interval set to {interval} ms.")  # Confirm interval set
                stdscr.refresh()
                time.sleep(1)
                stdscr.clear()
                stdscr.refresh()
                stdscr.addstr(0, 0, "Press '1' to start the sensor")
                stdscr.addstr(1, 0, "Press '2' to stop the sensor")
                stdscr.addstr(2, 0, "Press '3' to change the interval")
                stdscr.addstr(3, 0, "Press 'q' to quit the application.")
                stdscr.refresh()

            elif key == ord('q'):  # Quit on 'q'
                break

            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        curses.endwin()  # End curses mode
        rclpy.shutdown()


if __name__ == '__main__':
    main()
