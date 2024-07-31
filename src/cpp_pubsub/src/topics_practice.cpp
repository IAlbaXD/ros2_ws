#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(5), spiral_phase_(true), angle_(0.0), radius_(0.1)
    {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&MinimalPublisher::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1s, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (count_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Countdown: %zu", count_);
            count_--;
        } else {
            if (spiral_phase_) {
                RCLCPP_INFO(this->get_logger(), "Drawing spiral");
                draw_spiral();
            } else {
                RCLCPP_INFO(this->get_logger(), "Going straight");
                go_straight();
            }
        }
    }

    void draw_spiral()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = radius_;
        twist.angular.z = 1.0;
        radius_ += 0.1;         //(MODIFICADO 0.01)
        velocity_publisher_->publish(twist);
    }

    void go_straight()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 1.0;
        twist.angular.z = 0.0;
        velocity_publisher_->publish(twist);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        if (msg->x <= 1.0 || msg->x >= 8.0 || msg->y <= 1.0 || msg->y >= 8.0) {    //(MODIFICADO AREA)
            spiral_phase_ = false;                                                   //if (msg->x <= 1.0 || msg->x >= 10.0 || msg->y <= 1.0 || msg->y >= 10.0) {
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    size_t count_;
    bool spiral_phase_;
    double angle_;
    double radius_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
