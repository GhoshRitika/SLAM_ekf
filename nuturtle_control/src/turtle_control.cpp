/// \file TurtleControl
/// \brief This node enables control of the turtlebot via the cmd_vel topic.
///
/// PARAMETERS:
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The distance between the wheels.
///     motor_cmd_max(int): The motors are provided commands in [-motor_cmd_max, motor_cmd_max].
///     motor_cmd_per_rad_sec(double): Each motor command "tick" is 0.024 rad/sec.
///     encoder_ticks_per_rad(double): The number of encoder "ticks" per radian.
///
/// PUBLISHERS:
///     wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Publishes wheel velocities in ticks.
///     joint_states (sensor_msgs::msg::JointState): Provides the angle and velocity (rad/sec)
///
/// SUBSCRIBERS:
///     cmd_vel (geometry_msgs::msg::Twist): Receives the twist the robot should follow.
///     sensor_data (nuturtlebot_msgs::msg::SensorData): Receives robot wheel positions in ticks.


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"


using namespace std::chrono_literals;

/// \brief enables control of the turtlebot.
class TurtleControl : public rclcpp::Node
{
public:
  /// \brief declares all the node parameters, publishers and subscribers.
  TurtleControl()
  : Node("turtle_control"), count_(0)
  {
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
    declare_parameter("collision_radius", -1.0);
    wrad_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_ = get_parameter("track_width").get_parameter_value().get<double>();
    cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    cmd_prs_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    etsr_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    crad_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    if (wrad_ < 0 || track_ < 0 || cmd_max_ < 0 || cmd_prs_ < 0 || etsr_ < 0 || crad_ < 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Did not define a parameter");
      throw std::runtime_error("ERROR LOADING PARAMETERS");
    }

    wheelspeed_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    vel_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    sensor_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10,
      std::bind(&TurtleControl::sensorsub_callback, this, std::placeholders::_1));

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::twistsub_callback, this, std::placeholders::_1));
  }

private:
  /// @brief Receives the twist the robot should follow and publishes wheel velocites.
  /// @param msg the twist;
  void twistsub_callback(const geometry_msgs::msg::Twist & msg)
  {
    turtlelib::DiffDrive d(wrad_, track_);
    Vb_.w = msg.angular.z;
    Vb_.x = msg.linear.x;
    Vb_.y = msg.linear.y;
    phi_ = d.InvK(Vb_);
    w_ang_.left_velocity = static_cast<int>(phi_.left / cmd_prs_);
    w_ang_.right_velocity = static_cast<int>(phi_.right / cmd_prs_);
    if (w_ang_.left_velocity > cmd_max_) {
      w_ang_.left_velocity = cmd_max_;
    } else if (w_ang_.left_velocity < -cmd_max_) {
      w_ang_.left_velocity = -cmd_max_;
    }
    if (w_ang_.right_velocity > cmd_max_) {
      w_ang_.right_velocity = cmd_max_;
    } else if (w_ang_.right_velocity < -cmd_max_) {
      w_ang_.right_velocity = -cmd_max_;
    }
    wheelspeed_pub_->publish(w_ang_);
  }
  /// @brief Calculates the robot wheel positions and velocities and publishes the joint states.
  /// @param msg the left and right wheel positions in ticks.
  void sensorsub_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    wleft_ = (msg.left_encoder) / etsr_; //static_cast<double>(msg.left_encoder) / etsr_;
    wright_ = (msg.right_encoder) / etsr_; //static_cast<double>(msg.right_encoder) / etsr_;
    rclcpp::Time time = get_clock()->now();
    if (count_ == 0) {
      j_.header.frame_id = "blue/base_footprint";
      j_.header.stamp = time;
      j_.name = {"wheel_right_joint", "wheel_left_joint"};
      j_.position = {wright_, wleft_};
      j_.velocity = {0.0, 0.0};
      j_.effort = {};
    } else {
      double dt =
        ((time.seconds() + time.nanoseconds() * 1.0e-9) -
        (time_prev_.seconds() + time_prev_.nanoseconds() * 1.0e-9));
      j_.header.frame_id = "blue/base_footprint";
      j_.header.stamp = time;
      j_.name = {"wheel_right_joint", "wheel_left_joint"};
      j_.position = {wright_, wleft_};
      j_.velocity = {(wright_) / dt, (wleft_) / dt};
      j_.effort = {};
    }
    count_++;
    time_prev_ = time;
    vel_pub_->publish(j_);
  }

  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelspeed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr vel_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;
  int count_;
  double wrad_, track_, cmd_prs_, etsr_, crad_, wleft_ = 0.0, wright_ = 0.0, wleft_prev_ = 0.0,
    wright_prev_ = 0.0;
  int cmd_max_;
  turtlelib::Twist2D Vb_;
  turtlelib::Wheel_ang phi_;
  nuturtlebot_msgs::msg::WheelCommands w_ang_;
  sensor_msgs::msg::JointState j_;
  rclcpp::Time time_prev_;
};


/// \brief creates a turtle_control node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TurtleControl>());
  } catch (std::runtime_error & err) {
    std::cerr << err.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}
