#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "capra_control_msgs/msg/flippers.hpp"
#include "capra_control_msgs/msg/tracks.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

enum JoyButtons {
    // Buttons
    A = 0,
    B = 1,
    X = 3,
    Y = 4,
    LB = 6,
    RB = 7,
    VIEW = 10,
    MENU = 11,
    XBOX = 12,
    LS = 13,
    RS = 14,
    SHARE = 15,
};

enum JoyAxes {
    // Axes
    LS_X = 0,
    LS_Y = 1,
    LT = 5,
    RS_X = 2,
    RS_Y = 3,
    RT = 4,
    D_PAD_X = 6,
    D_PAD_Y = 7,
};

class MapperNode : public rclcpp::Node
{
public:
    using Twist = geometry_msgs::msg::Twist;
    using Joy = sensor_msgs::msg::Joy;
    using Bool = std_msgs::msg::Bool;
    using Flippers = capra_control_msgs::msg::Flippers;
    using Tracks = capra_control_msgs::msg::Tracks;

    MapperNode()
        : Node("actions_mapper")
    {
        // Declare parameters
        spacing_ = this->declare_parameter("wheel_separation", 0.175);

        // Create subcriptions
        cmdVelSub_ = this->create_subscription<Twist>(
            "cmd_vel", 1, std::bind(&MapperNode::cmd_vel_cb, this, std::placeholders::_1));
        joySub_ = this->create_subscription<Joy>(
            "joy", 1, std::bind(&MapperNode::joy_cb, this, std::placeholders::_1));

        // Create publishers
        enablePub_ = this->create_publisher<Bool>("enable", 1);
        estopPub_ = this->create_publisher<Bool>("estop", 1);
        flippersPub_ = this->create_publisher<Flippers>("flippers_cmd", 1);
        tracksPub_ = this->create_publisher<Tracks>("tracks_cmd", 1);
    }

private:
    void cmd_vel_cb(Twist::SharedPtr cmd_vel)
    {
        auto tracksMsg = Tracks();
        tracksMsg.left  = cmd_vel->linear.x - (cmd_vel->angular.z * spacing_ / 2);
        tracksMsg.right = cmd_vel->linear.x + (cmd_vel->angular.z * spacing_ / 2);

        tracksPub_->publish(tracksMsg);
    }

    void joy_cb(Joy::SharedPtr joy)
    {
        enable_ = joy->buttons[JoyButtons::A];
        if (joy->buttons[JoyButtons::B]) estop_ = false;
        if (joy->buttons[JoyButtons::XBOX]) estop_ = true;

        float flipperSpeed = joy->axes[JoyAxes::D_PAD_Y];

        auto flippers = Flippers();

        if (joy->buttons[JoyButtons::LB]) flippers.front_left = flipperSpeed;
        if (joy->buttons[JoyButtons::RB]) flippers.front_right = flipperSpeed;
        if (joy->axes[JoyAxes::LT] < 0) flippers.rear_left = flipperSpeed;
        if (joy->axes[JoyAxes::RT] < 0) flippers.rear_right = flipperSpeed;

        auto enableMsg = Bool();
        enableMsg.data = enable_;
        enablePub_->publish(enableMsg);

        auto estopMsg = Bool();
        estopMsg.data = estop_;
        estopPub_->publish(estopMsg);

        flippersPub_->publish(flippers);
    }

    bool enable_;
    bool estop_;
    float spacing_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<Twist>::SharedPtr cmdVelSub_;
    rclcpp::Subscription<Joy>::SharedPtr joySub_;

    rclcpp::Publisher<Bool>::SharedPtr enablePub_;
    rclcpp::Publisher<Bool>::SharedPtr estopPub_;
    rclcpp::Publisher<Flippers>::SharedPtr flippersPub_;
    rclcpp::Publisher<Tracks>::SharedPtr tracksPub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}