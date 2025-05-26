#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

class FlockingNode : public rclcpp::Node
{
public:
    FlockingNode()
        : Node("flocking_node")
    {
        // Create a publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/velocity_commands", 10);

        // Create a subscriber for agent positions
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/agent_positions", 10, std::bind(&FlockingNode::position_callback, this, std::placeholders::_1));
    }

private:
void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // Store agent positions
    agent_positions_[this->get_name()] = {msg->x, msg->y, msg->z};
    RCLCPP_INFO(this->get_logger(), "Received agent position: [%f, %f, %f]", msg->x, msg->y, msg->z);

    if (agent_positions_.size() > 1)
    {
        // Calculate flocking behavior
        auto velocity = calculate_flocking();
        RCLCPP_INFO(this->get_logger(), "Publishing velocity: [%f, %f, %f]", velocity.linear.x, velocity.linear.y, velocity.linear.z);

        // Publish velocity command
        publisher_->publish(velocity);
    }
}

    geometry_msgs::msg::Twist calculate_flocking()
    {
        // Example flocking logic (separation, alignment, cohesion)
        auto separation = calculate_separation();
        auto alignment = calculate_alignment();
        auto cohesion = calculate_cohesion();

        // Combine behaviors into a velocity command
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = separation[0] + alignment[0] + cohesion[0];
        velocity.linear.y = separation[1] + alignment[1] + cohesion[1];
        velocity.linear.z = separation[2] + alignment[2] + cohesion[2];
        return velocity;
    }

    std::vector<double> calculate_separation()
    {
        // separation 
        return {0.1, 0.1, 0.0}; // agent positions
    }

    std::vector<double> calculate_alignment()
    {
        // alignment 
        return {0.1, 0.1, 0.0}; // agent headings
    }

    std::vector<double> calculate_cohesion()
    {
        //  cohesion 
        return {0.1, 0.1, 0.0}; //  agent positions
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    std::unordered_map<std::string, std::vector<double>> agent_positions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlockingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
