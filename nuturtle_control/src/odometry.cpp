/// \file Odometry
/// \brief This node publishes odometry messages and the odometry transform.
///
/// PARAMETERS:
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The distance between the wheels.
///     body_id(string): The name of the body frame of the robot.
///     odom_id(string): The name of the odometry frame..
///     wheel_left(string): The name of the left wheel joint.
///     wheel_right(string): The name of the right wheel joint.
///
/// PUBLISHERS:
///     odom(nav_msgs::msg::Odometry): Updates the pose and twist of the robot relative to odom.
///     blue/nav_path (nav_msgs::msg::Path): Publishes the blue robot path.
///
/// SUBSCRIBERS:
///     joint_states(sensor_msgs::msg::JointState): Receives the wheel positions and updates the
///     internal odometry.
///
/// SERVERS:
///     intial_pose(nuturtle_control::srv::InitialPose): Gives the new configuration for the robot.

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
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;

/// \brief publishes the odometry messages and odometry transform
class Odometry : public rclcpp::Node
{
public:
  /// \brief declares all the node parameters. publishers, subscribers, services and transforms
  Odometry()
  : Node("odometry")
  {
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("body_id", "unspecified");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "unspecified");
    declare_parameter("wheel_right", "unspecified");
    wrad_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_ = get_parameter("track_width").get_parameter_value().get<double>();
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();


    if (body_id_ == "unspecified" || wheel_left_ < "unspecified" || wheel_right_ < "unspecified") {
      RCLCPP_ERROR_STREAM(get_logger(), "Unspecified frames");
      throw std::runtime_error("UNSPECIFIED FRAMES");
    }
    if (wrad_ < 0 || track_ < 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Did not define a parameter");
      throw std::runtime_error("ERROR LOADING PARAMETERS");
    }


    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    internal_odom_ = turtlelib::DiffDrive{wrad_, track_};
    q_ = internal_odom_.get_config();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Odometry::jointstatesub_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    init_server_ =
      create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(&Odometry::init_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    path_pub_ = create_publisher<nav_msgs::msg::Path>("blue/nav_path", 10);
  }

private:
  /// @brief Calculates the new twist and updated configuration to publish the odom and transform.
  /// @param msg - Provides the position of the wheel.
  void jointstatesub_callback(const sensor_msgs::msg::JointState & msg)
  {
    odom_.header.stamp = get_clock()->now();

    wang_.right = msg.position.at(0) - wright_prev_;
    wang_.left = msg.position.at(1) - wleft_prev_;

    Vb_ = internal_odom_.FwdK(wang_);
    q_ = internal_odom_.get_config();

    odom_.pose.pose.position.x = q_.x;
    odom_.pose.pose.position.y = q_.y;
    tf2::Quaternion theta;
    theta.setRPY(0, 0, q_.theta);
    odom_.pose.pose.orientation.x = theta.x();
    odom_.pose.pose.orientation.y = theta.y();
    odom_.pose.pose.orientation.z = theta.z();
    odom_.pose.pose.orientation.w = theta.w();
    odom_.twist.twist.linear.x = Vb_.x;
    odom_.twist.twist.linear.y = Vb_.y;
    odom_.twist.twist.angular.z = Vb_.w;
    wright_prev_ = msg.position.at(0);
    wleft_prev_ = msg.position.at(1);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;
    t.transform.translation.x = q_.x;
    t.transform.translation.y = q_.y;
    t.transform.rotation.x = theta.x();
    t.transform.rotation.y = theta.y();
    t.transform.rotation.z = theta.z();
    t.transform.rotation.w = theta.w();
    tf_broadcaster_->sendTransform(t);
    odom_pub_->publish(odom_);

    path_.header.stamp = get_clock()->now();
    path_.header.frame_id = odom_id_;
    pstmp_.pose.position.x = q_.x;
    pstmp_.pose.position.y = q_.y;
    pstmp_.pose.orientation.x = theta.x();
    pstmp_.pose.orientation.y = theta.y();
    pstmp_.pose.orientation.z = theta.z();
    pstmp_.pose.orientation.w = theta.w();
    path_.poses.push_back(pstmp_);
    path_pub_->publish(path_);
  }

  /// @brief The location of the odometry is reset to requested configuration.
  /// @param request - This message has the x, y and theta values of the new configuration.
  void init_callback(
    nuturtle_control::srv::InitialPose::Request::SharedPtr request,
    nuturtle_control::srv::InitialPose::Response::SharedPtr)
  {

    internal_odom_ = turtlelib::DiffDrive{{request->theta, request->x, request->y},
      {0.0, 0.0}, wrad_, track_};
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr init_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  nav_msgs::msg::Odometry odom_;
  turtlelib::DiffDrive internal_odom_;
  double wrad_, track_, wleft_prev_ = 0.0, wright_prev_ = 0.0;
  turtlelib::Wheel_ang wang_;
  turtlelib::Config q_;
  turtlelib::Twist2D Vb_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pstmp_;
};


/// @brief creates an odometry node.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Odometry>());
  } catch (std::runtime_error & err) {
    std::cerr << err.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}
