#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>

using namespace std::chrono_literals;

class SensorServer : public rclcpp::Node {
public:
    SensorServer()
        : Node("sensor_server"), interval_(1000) {
        // Declare parameter for the interval (in milliseconds)
        this->declare_parameter("interval", interval_);
        this->get_parameter("interval", interval_);

        // Create publishers for various sensor data
        voltage_pub_ = this->create_publisher<std_msgs::msg::String>("sensor/supply_voltage", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("sensor/env_temp", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::String>("sensor/yaw", 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::String>("sensor/pitch", 10);
        roll_pub_ = this->create_publisher<std_msgs::msg::String>("sensor/roll", 10);

        // Setup the TCP server socket
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(2000);
        bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr));
        listen(server_socket_, 1);
        RCLCPP_INFO(this->get_logger(), "Sensor Server started, waiting for client...");

        // Wait for a connection from the client
        socklen_t addr_len = sizeof(client_addr_);
        client_socket_ = accept(server_socket_, (struct sockaddr*)&client_addr_, &addr_len);
        RCLCPP_INFO(this->get_logger(), "Client connected: %s", inet_ntoa(client_addr_.sin_addr));

        // Timer for periodic sensor status updates
        timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_), std::bind(&SensorServer::send_status, this));

        // Create services for starting and stopping the sensor
        start_service_ = this->create_service<std_srvs::srv::SetBool>("start_sensor", std::bind(&SensorServer::start_sensor, this, std::placeholders::_1, std::placeholders::_2));
        stop_service_ = this->create_service<std_srvs::srv::SetBool>("stop_sensor", std::bind(&SensorServer::stop_sensor, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~SensorServer() {
        close(client_socket_);
        RCLCPP_INFO(this->get_logger(), "Client connection closed.");
        close(server_socket_);
    }

private:
    void send_status() {
        // Simulated sensor data
        int supply_voltage = 3300;  // in milli-volts
        int env_temp = 250;          // in deci-Celsius
        int yaw = 5;                 // in deci-degrees
        int pitch = 3;               // in deci-degrees
        int roll = 10;               // in deci-degrees

        // Publish the data to ROS topics
        publish_data(supply_voltage, "supply_voltage");
        publish_data(env_temp, "env_temp");
        publish_data(yaw, "yaw");
        publish_data(pitch, "pitch");
        publish_data(roll, "roll");

        // Send status message back to the client
        auto status_message = create_status_message(supply_voltage, env_temp, yaw, pitch, roll);
        send(client_socket_, status_message.c_str(), status_message.size(), 0);
    }

    void publish_data(int value, const std::string& topic_name) {
        if (topic_name == "supply_voltage") {
            auto message = std_msgs::msg::String();
            message.data = std::to_string(value) + "mV";
            voltage_pub_->publish(message);
        } else if (topic_name == "env_temp") {
            auto message = sensor_msgs::msg::Temperature();
            message.temperature = value / 10.0;  // Convert to Celsius
            temp_pub_->publish(message);
        } else {
            auto message = std_msgs::msg::String();
            message.data = std::to_string(value / 10.0) + " degrees";
            if (topic_name == "yaw") {
                yaw_pub_->publish(message);
            } else if (topic_name == "pitch") {
                pitch_pub_->publish(message);
            } else if (topic_name == "roll") {
                roll_pub_->publish(message);
            }
        }
    }

    std::string create_status_message(int supply_voltage, int env_temp, int yaw, int pitch, int roll) {
        // Encode the status message in hexadecimal format
        std::string message = "Start11";
        message += encode_value(supply_voltage);
        message += encode_value(env_temp);
        message += encode_value(yaw);
        message += encode_value(pitch);
        message += encode_value(roll);
        message += "End\r\n";
        return message;
    }

    std::string encode_value(int value) {
        // Little endian encoding of 16-bit integer to hex string
        char buffer[5];
        snprintf(buffer, sizeof(buffer), "%04X", value);
        return std::string(buffer);
    }

    void start_sensor(const std::shared_ptr<std_srvs::srv::SetBool::Request> /*request*/,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Starting sensor...");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_), std::bind(&SensorServer::send_status, this));
        response->success = true;
        response->message = "Sensor started.";
    }

    void stop_sensor(const std::shared_ptr<std_srvs::srv::SetBool::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Stopping sensor...");
        timer_ = nullptr;  // Resetting the timer to stop it
        response->success = true;
        response->message = "Sensor stopped.";
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voltage_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr roll_pub_;
    int interval_;
    int server_socket_;
    int client_socket_;
    struct sockaddr_in client_addr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorServer>());
    rclcpp::shutdown();
    return 0;
}
