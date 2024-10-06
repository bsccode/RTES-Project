#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>

class BallColorDetector : public rclcpp::Node {
public:
    BallColorDetector() : Node("ball_color_detector") {
        // Create publisher for ball color and coordinates
        ball_info_pub_ = this->create_publisher<std_msgs::msg::String>("ball_info", 10);

        // Subscribe to camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&BallColorDetector::process_image, this, std::placeholders::_1)
        );
    }

private:
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to HSV
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        std_msgs::msg::String ball_info_msg;
        ball_info_msg.data = "None"; // Default value, no ball detected

        // Reset the coordinates for this cycle
        float ball_center_x = -1.0;
        float ball_center_y = -1.0;

        // Detect red ball
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv_image, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), red_mask1); // Lower red
        cv::inRange(hsv_image, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), red_mask2); // Upper red
        red_mask = red_mask1 | red_mask2; // Combine both ranges

        // Detect green ball
        cv::Mat green_mask;
        cv::inRange(hsv_image, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), green_mask);

        // Detect blue ball
        cv::Mat blue_mask;
        cv::inRange(hsv_image, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), blue_mask);

        // Combine all masks into a single image for display
        cv::Mat combined_mask;
        cv::bitwise_or(red_mask, green_mask, combined_mask);
        cv::bitwise_or(combined_mask, blue_mask, combined_mask);

        // Show the combined mask in a single window
        cv::imshow("Color Masks", combined_mask);
        cv::waitKey(1); // Refresh the window

        // Detect red ball
        if (is_ball_detected(red_mask, ball_center_x, ball_center_y)) {
            ball_info_msg.data = "Red," + std::to_string(ball_center_x) + "," + std::to_string(ball_center_y);
        }
        // Detect green ball
        else if (is_ball_detected(green_mask, ball_center_x, ball_center_y)) {
            ball_info_msg.data = "Green," + std::to_string(ball_center_x) + "," + std::to_string(ball_center_y);
        }
        // Detect blue ball
        else if (is_ball_detected(blue_mask, ball_center_x, ball_center_y)) {
            ball_info_msg.data = "Blue," + std::to_string(ball_center_x) + "," + std::to_string(ball_center_y);
        }

        // Publish the ball color and coordinates
        ball_info_pub_->publish(ball_info_msg);
    }

    // Helper function to detect a ball and extract its coordinates
    bool is_ball_detected(const cv::Mat &mask, float &center_x, float &center_y) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) { // Minimum area threshold for detection
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contour, center, radius);

                // Check roundness
                if (std::fabs(1 - ((double)radius * radius * M_PI) / area) <= 0.2) {
                    // Set the center coordinates of the detected ball
                    center_x = center.x;
                    center_y = center.y;
                    return true; // Ball detected
                }
            }
        }
        return false; // No ball detected
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BallColorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
