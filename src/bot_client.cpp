#include <iostream>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <curses.h>

class TcpClientNode : public rclcpp::Node
{
public:
    TcpClientNode()
        : Node("bot_client_node"), stop_requested(false)
    {
        // Create a TCP client socket
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
            exit(1);
        }

        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(2000); // Server port
        server_address.sin_addr.s_addr = inet_addr("127.0.0.1"); // Server address

        // Connect to the server
        if (connect(sockfd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed!");
            exit(1);
        }

        // Create a publisher to send messages to a ROS topic (optional)
        pub_ = this->create_publisher<std_msgs::msg::String>("teleop_status", 10);

        RCLCPP_INFO(this->get_logger(), "TCP Client Node initialized. Use keys for control: ");
        RCLCPP_INFO(this->get_logger(), "'i' = Move Forward, 'j' = Turn Left, 'm' = Move Backward, 'l' = Turn Right, 'k' = Stop");

        teleoperate();
    }

    ~TcpClientNode()
    {
        // Close the socket
        close(sockfd);
    }

private:
    int sockfd;
    struct sockaddr_in server_address;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    bool stop_requested;

    void send_command(const std::string &command)
    {
        // Send the command to the server
        send(sockfd, command.c_str(), command.length(), 0);

        // Optionally, log the sent command to a ROS topic
        auto msg = std_msgs::msg::String();
        msg.data = "Sent command: " + command;
        pub_->publish(msg);
    }

    void teleoperate()
    {
        // Initialize curses screen
        initscr();
        cbreak();
        keypad(stdscr, TRUE);
        noecho();

        try {
            while (!stop_requested) {
                int key = getch();  // Get the pressed key

                if (key == 'i') {  // Move forward
                    send_command("move_forward");
                }
                else if (key == 'j') {  // Turn left
                    send_command("turn_left");
                }
                else if (key == 'm') {  // Move backward
                    send_command("move_backward");
                }
                else if (key == 'l') {  // Turn right
                    send_command("turn_right");
                }
                else if (key == 'k') {  // Stop the robot
                    send_command("stop");
                }
                else if (key == 'q') {  // Quit (exit) the program
                    RCLCPP_INFO(this->get_logger(), "Exiting control loop.");
                    stop_requested = true;
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }

        // Correctly end the curses session after the try block
        endwin();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpClientNode>());
    rclcpp::shutdown();
    return 0;
}
