#include <iostream>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TcpServerNode : public rclcpp::Node
{
public:
    TcpServerNode()
        : Node("bot_server_node")
    {
        // Create a TCP server socket
        server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sockfd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
            exit(1);
        }

        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(2000);  // Port
        server_address.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces

        if (bind(server_sockfd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed!");
            exit(1);
        }

        if (listen(server_sockfd, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen failed!");
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "TCP Server Node started. Waiting for connection...");

        // Create a publisher for turtlesim movement
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Accept a connection from the client
        int client_sockfd = accept(server_sockfd, NULL, NULL);
        if (client_sockfd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Accept failed!");
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "Client connected. Waiting for commands...");

        // Start receiving commands from the client
        handle_client(client_sockfd);
    }

    ~TcpServerNode()
    {
        close(server_sockfd);
    }

private:
    int server_sockfd;
    struct sockaddr_in server_address;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    void send_command(const std::string &command)
    {
        auto twist_msg = geometry_msgs::msg::Twist();

        // Send appropriate motion commands to the turtle
        if (command == "move_forward") {
            twist_msg.linear.x = 1.0;
        }
        else if (command == "move_backward") {
            twist_msg.linear.x = -1.0;
        }
        else if (command == "turn_left") {
            twist_msg.angular.z = 1.0;
        }
        else if (command == "turn_right") {
            twist_msg.angular.z = -1.0;
        }
        else if (command == "stop") {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
        }

        pub_->publish(twist_msg);
    }

    void handle_client(int client_sockfd)
    {
        char buffer[1024];
        while (true) {
            int read_size = read(client_sockfd, buffer, sizeof(buffer));
            if (read_size == 0) {
                RCLCPP_INFO(this->get_logger(), "Client disconnected.");
                break;
            }

            buffer[read_size] = '\0';
            std::string command(buffer);

            RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

            // Send the command to control the turtle
            send_command(command);
        }
        close(client_sockfd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpServerNode>());
    rclcpp::shutdown();
    return 0;
}
