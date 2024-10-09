#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

class MoveForwardServiceNode : public rclcpp::Node
{
public:
    MoveForwardServiceNode()
    : Node("move_forward_service_node")
    {
        service_ = this->create_service<std_srvs::srv::Empty>(
            "move_forward",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                this->handle_service(request_header, request, response);
            });

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Move Forward Service Node started.");
    }

private:
    void handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Move Forward Service called.");
        moveForwardAndStop();
    }

    void moveForwardAndStop() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.15;  // Move forward
        cmd_publisher_->publish(twist_msg);

        // Sleep for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));

        twist_msg.linear.x = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveForwardServiceNode>());
    rclcpp::shutdown();
    return 0;
}
