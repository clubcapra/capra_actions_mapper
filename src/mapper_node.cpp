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
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    VIEW = 6,
    MENU = 7,
    XBOX = 8,
    LS = 9,
    RS = 10,
    SHARE = 11,
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
        turnMultiplier_ = this->declare_parameter("turn_multiplier", 10.0);

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
        tracksMsg.left  = cmd_vel->linear.x - (turnMultiplier_ * cmd_vel->angular.z * spacing_ / 2);
        tracksMsg.right = cmd_vel->linear.x + (turnMultiplier_ * cmd_vel->angular.z * spacing_ / 2);

        tracksPub_->publish(tracksMsg);
    }

    void joy_cb(Joy::SharedPtr joy)
    {
        enable_ = joy->buttons[JoyButtons::A];
        if (joy->buttons[JoyButtons::B]) estop_ = false;
        if (joy->buttons[JoyButtons::XBOX]) estop_ = true;

        float flipperSpeed = joy->axes[JoyAxes::D_PAD_Y];

        auto flippers = Flippers();

        bool front_left_selected = joy->buttons[JoyButtons::LB];
        bool front_right_selected = joy->buttons[JoyButtons::RB];
        bool rear_left_selected = joy->axes[JoyAxes::LT] < 0;
        bool rear_right_selected = joy->axes[JoyAxes::RT] < 0;
        bool all_selected = !(front_left_selected || front_right_selected || rear_left_selected || rear_right_selected);

        if (front_left_selected || all_selected) flippers.front_left = flipperSpeed;
        if (front_right_selected || all_selected) flippers.front_right = flipperSpeed;
        if (rear_left_selected || all_selected) flippers.rear_left = flipperSpeed;
        if (rear_right_selected || all_selected) flippers.rear_right = flipperSpeed;

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
    float turnMultiplier_;

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