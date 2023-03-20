// publishes cmd_vel commands to cause the robot to drive in a circle of a specified radius at a specified speed.
/// \file Circle
/// \brief This node makes the robot drive in a circle.
///
/// PARAMETERS:
///     frequency (int):  Rate at which the cmd_vel topic is published.
///
/// PUBLISHERS:
///     cmd_vel(geometry_msgs::msg::Twist): The linear and angular twist at which the robot moves.
///
/// SERVERS:
///     control(): Recives the speed and radius of the circle the robot will drive.
///     reverse(std_srvs/srv/Empty): Reverses the direction of the robot along the arc.
///     stop(std_srvs/srv/Empty): Stops the robot.


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"


using namespace std::chrono_literals;

/// \brief publishes velocity to move in a circle
class Circle : public rclcpp::Node
{
public:
  /// \brief declares all the node parameter, publishers and services.
  Circle()
  : Node("circle"), flag_(true), rad_(0.0), speed_(0.0)
  {
    declare_parameter("frequency", 100);
    int frequency = get_parameter("frequency").get_parameter_value().get<int>();
    auto freq = 1s / frequency;
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    control_server_ =
      create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));

    reverse_server_ =
      create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));

    stop_server_ =
      create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(freq, std::bind(&Circle::timer_callback, this));
  }

private:
  /// @brief Publish the cmd_vel at fixed rate.
  void timer_callback()
  {
    if (flag_ == true) {
      Vb_.linear.x = rad_ * speed_;
      Vb_.linear.y = 0.0;
      Vb_.angular.z = speed_;
      vel_pub_->publish(Vb_);
      flag_ = false;
    }
  }
  /// @brief Starts moving the robot in a circle of specified radius and speed.
  /// @param request - Provides the radius and speed of the circle robot drives.
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
    flag_ = true;
    rad_ = request->radius;
    speed_ = request->velocity;
  }

  /// @brief Reverses the direction of circular motion.
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    flag_ = true;
    speed_ = -speed_;
  }

  /// @brief makes the robot stop moving.
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    flag_ = false;
    Vb_.linear.x = 0.0;
    Vb_.linear.y = 0.0;
    Vb_.angular.z = 0.0;
    vel_pub_->publish(Vb_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  bool flag_;
  double rad_, speed_;
  geometry_msgs::msg::Twist Vb_;
};


/// @brief CReates a circle node.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();

  return 0;
}
