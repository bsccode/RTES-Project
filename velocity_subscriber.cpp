#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

class BallInfoSubscriber : public rclcpp::Node {
public:
    BallInfoSubscriber() : Node("ball_info_subscriber") {
        // Subscribe to ball_info (color and coordinates)
        ball_info_sub_ = this->create_subscription<std_msgs::msg::String>(
            "ball_info", 10,
            std::bind(&BallInfoSubscriber::ball_info_callback, this, std::placeholders::_1)
        );
    }

private:
    void ball_info_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data != "None") {
            std::istringstream ss(msg->data);
            std::string color;
            float x, y;

            std::getline(ss, color, ',');
            ss >> x;
            ss.ignore(1); // Skip the comma
            ss >> y;

            RCLCPP_INFO(this->get_logger(), "Ball detected - Color: %s, Coordinates: X: %.2f, Y: %.2f", color.c_str(), x, y);
        } else {
            RCLCPP_INFO(this->get_logger(), "No ball detected.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_info_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallInfoSubscriber>());
    rclcpp::shutdown();
    return 0;
}

