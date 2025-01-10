## Problem Statement -

1. Automatically sends the start command (with the interval that can be set as a parameter)
to the sensor on launch
2. Decodes the status messages received and publishes the various parameters to
appropriate topics. Specify the names and data types of the topics.
3. Provide an option to the user start (with custom interval) and stop the sensor. Specify
how this is done.

Taking the above 3 problem statements into consideration I implemented scripts in both python and cpp - 

 # Python Implementation - 

1.sensor_server.py - This script is a basic simulation of a sensor server that communicates with a client over TCP while publishing sensor data to ROS 2 topics. It supports the functionality to send periodic data (with a custom interval) and allows the simulation of sensor data like voltage, temperature, and orientation parameters (yaw, pitch, roll).

    ros2 run tcp_examples sensor_server.py

2.sensor_client.py - This script connects to a sensor server, sends a start command with an interval, receives and decodes sensor data.

    ros2 run tcp_examples sensor_client.py
    
To Start or Stop the sensor -

    ros2 service call /start_sensor std_srvs/srv/SetBool "data: true"
    ros2 service call /stop_sensor std_srvs/srv/SetBool "data: false"
    ros2 param set /sensor_server interval 2000 #set your value

3.set_custom_value.py - To run the above 3 steps in a single click I implemented user friendly python script gides the user to run the available choices.


     ros2 run tcp_examples set_custom_value.py

If you want to run the both sensor_server.py and sensor_client.py there is a launch file with set interval

     ros2 launch tcp_examples cpp_sensor.launch.py
     ros2 launch tcp_examples py_sensor.launch.py

# cpp Implementation -

     ros2 run tcp_examples sensor_server
     ros2 run tcp_examples sensor_client

Here you can use the python script in cpp to set the custom value -

     ros2 run tcp_examples set_custom_value.py

 # demo_tcp - 

 Here I implemented one more script that controls the turtlebot over tcp protocol in both python and cpp

 To run the python implementation follow the below cmd

     ros2 run turtlesim turtlesim_node 
     ros2 run tcp_examples bot_server.py
     ros2 run tcp_examples bot_client.py

To run the cpp implementation follow the below cmd

     ros2 run turtlesim turtlesim_node 
     ros2 run tcp_examples bot_server
     ros2 run tcp_examples bot_client

