#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/set_mode.hpp"  // Include the service definition
#include <iostream>
#include <cstdlib> // For std::atoi

class WaypointCounter : public rclcpp::Node
{
public:
    WaypointCounter(int number) : Node("waypoint_counter"), target_wp_(number), wp_completed_count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Total Waypoints: %d", target_wp_);

        // Create the service client for /mavros/set_mode
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // Subscribe to MAVROS waypoint reached topic
        wp_reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
            "/mavros/mission/reached", 10, 
            std::bind(&WaypointCounter::wpReachedCallback, this, std::placeholders::_1));
    }

private:
    // Callback function triggered when a waypoint is reached
    void wpReachedCallback(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
    {
        wp_completed_count_++; // Increment count on waypoint reached

        RCLCPP_INFO(this->get_logger(), "Waypoint Reached: %d, Total Completed: %d", msg->wp_seq, wp_completed_count_);

        // Compare wp_seq with the input number
        if (msg->wp_seq == target_wp_)
        {
            RCLCPP_INFO(this->get_logger(), "Switching to Guided");
            switchToGuided();
        }
    }

    void switchToGuided()
    {
        // Create request
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;  // Not used in ROS 2
        request->custom_mode = "GUIDED";

        // Send request asynchronously
        auto future = set_mode_client_->async_send_request(request);

        // Wait for response
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully switched to GUIDED mode.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch to GUIDED mode.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out.");
        }
    }

    int target_wp_;  // The input waypoint number to compare against
    int wp_completed_count_;  // Stores the count of waypoints reached
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr wp_reached_sub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;  // Service client for set_mode
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cerr << "Usage: ros2 run my_package waypoint_counter <integer>\n";
        return 1;
    }

    int input_number = std::atoi(argv[1]);

    rclcpp::spin(std::make_shared<WaypointCounter>(input_number));

    rclcpp::shutdown();
    return 0;
}
