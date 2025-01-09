#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <iomanip>

class SensorClient : public rclcpp::Node {
public:
    SensorClient()
        : Node("sensor_client"), interval_(1000), data_received_(false) {
        // Declare parameter for the interval (in milliseconds)
        this->declare_parameter("interval", interval_);
        this->get_parameter("interval", interval_);

        // Create a TCP connection to the server
        server_ip_ = "127.0.0.1";
        server_port_ = 2000;
        connect_to_server();

        // Send Start command
        send_start_command();
    }

    ~SensorClient() {
        stop();
    }

    void receive_status() {
        char buffer[1024];
        ssize_t len = recv(server_socket_, buffer, sizeof(buffer) - 1, 0);
        if (len > 0) {
            buffer[len] = '\0'; // Null-terminate the received data
            decode_and_publish_status(buffer);
        }
    }

private:
    void connect_to_server() {
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);
        inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr);

        if (connect(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection to server failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to server");
    }

    void send_start_command() {
        // Command ID for Start Message = 03, Interval (in milliseconds)
        std::string command = "Start03";
        uint16_t interval = static_cast<uint16_t>(interval_);
        std::ostringstream payload_stream;
        payload_stream.write(reinterpret_cast<const char*>(&interval), sizeof(interval)); // Little-endian encoding of interval
        std::string payload = payload_stream.str();
        std::string command_message = command + to_hex(payload) + "End\r\n";
        
        send(server_socket_, command_message.c_str(), command_message.size(), 0);
        RCLCPP_INFO(this->get_logger(), "Sent Start Command");
    }

    std::string to_hex(const std::string& input) {
        std::ostringstream oss;
        for (unsigned char c : input) {
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
        }
        return oss.str();
    }

    void decode_and_publish_status(const char* data) {
        // Log the received data only once
        if (!data_received_) {
            RCLCPP_INFO(this->get_logger(), "Received Data: %s", data);
            data_received_ = true; // Set the flag to true after logging
        }
    }

    void stop() {
        // Send Stop command
        std::string command = "Start09End\r\n";
        send(server_socket_, command.c_str(), command.size(), 0);
        RCLCPP_INFO(this->get_logger(), "Sent Stop Command");
        close(server_socket_);
    }

    int server_socket_;
    std::string server_ip_;
    int server_port_;
    int interval_;
    bool data_received_; // Flag to track if data has been received
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorClient>();

    try {
        while (rclcpp::ok()) {
            node->receive_status();
            rclcpp::spin_some(node);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Unknown exception occurred");
    }

    rclcpp::shutdown();
    return 0;
}