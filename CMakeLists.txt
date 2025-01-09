cmake_minimum_required(VERSION 3.8)
project(tcp_examples)

# Compiler flags for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add C++ TCP server executable
add_executable(bot_server src/bot_server.cpp)
ament_target_dependencies(bot_server rclcpp std_msgs geometry_msgs Boost)

# Add C++ TCP client executable
add_executable(bot_client src/bot_client.cpp)
ament_target_dependencies(bot_client rclcpp std_msgs Boost geometry_msgs)

add_executable(sensor_server src/sensor_server.cpp)
ament_target_dependencies(sensor_server rclcpp std_msgs sensor_msgs std_srvs Boost)

add_executable(sensor_client src/sensor_client.cpp)
ament_target_dependencies(sensor_client rclcpp std_msgs sensor_msgs std_srvs Boost)

# Link curses library to bot_client (used for terminal input handling)
target_link_libraries(bot_client curses)

# Install C++ executables
install(TARGETS
  bot_server
  bot_client
  sensor_server
  sensor_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python TCP server and client scripts
install(PROGRAMS
  ${PROJECT_NAME}/bot_server.py
  ${PROJECT_NAME}/bot_client.py
  ${PROJECT_NAME}/sensor_server.py
  ${PROJECT_NAME}/sensor_client.py
  ${PROJECT_NAME}/set_custom_value.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch src tcp_examples
  DESTINATION share/${PROJECT_NAME}
)

# For Python dependencies
ament_python_install_package(${PROJECT_NAME})

# Final package
ament_package()
