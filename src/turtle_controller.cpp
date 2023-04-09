#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/char.hpp"
class TurtleController : public rclcpp::Node {
public:
  TurtleController() : Node("turtle_controller") {
    // Declare parameters for controlling the turtle
    declare_parameter("go_forward_key", 'w');
    declare_parameter("go_backward_key", 's');
    declare_parameter("turn_left_key", 'a');
    declare_parameter("turn_right_key", 'd');
    get_parameter("go_forward_key", go_forward_key_);
    get_parameter("go_backward_key", go_backward_key_);
    get_parameter("turn_left_key", turn_left_key_);
    get_parameter("turn_right_key", turn_right_key_);

    // Initialize the publisher for sending twist messages to the turtle
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    // Initialize the timer for publishing twist messages
    auto timer_callback = std::bind(&TurtleController::publish_twist, this);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), timer_callback);

    // Initialize the keyboard input subscriber
    auto key_callback = std::bind(&TurtleController::keyboard_input_callback, this, std::placeholders::_1);
    key_sub_ = create_subscription<std_msgs::msg::Char>("keyboard_input", 10, key_callback);
  }

private:
  // Callback function for receiving keyboard input
  void keyboard_input_callback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;
    if (key == go_forward_key_) {
      linear_velocity_ = 1.0;
    } else if (key == go_backward_key_) {
      linear_velocity_ = -1.0;
    } else if (key == turn_left_key_) {
      angular_velocity_ = 1.0;
    } else if (key == turn_right_key_) {
      angular_velocity_ = -1.0;
    }
  }

  // Function for publishing twist messages to the turtle
  void publish_twist() {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_velocity_;
    twist_msg.angular.z = angular_velocity_;
    twist_pub_->publish(twist_msg);
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
  }

  // ROS2 parameters for controlling the turtle
  char go_forward_key_;
  char go_backward_key_;
  char turn_left_key_;
  char turn_right_key_;

  // ROS2 objects for communication with the turtle
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Velocity values for the turtle
  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
