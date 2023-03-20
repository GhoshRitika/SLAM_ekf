/// \file Slam
/// \brief This node implements the Extended Kalman Filter and visualizes the localized robot
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
///     landmarks(visualization_msgs::msg::MarkerArray): Publishes the estimated landmark markers
///     green/nav_path(nav_msgs::msg::Path): Publishes the path followed by the SLAM robot
///
/// SUBSCRIBERS:
///     joint_states(sensor_msgs::msg::JointState): Receives the wheel positions and updates the
///     internal odometry.
///     fake_sensor(visualization_msgs::msg::MarkerArray): Receives the position of obstacle relative to the robot
///
/// SERVERS:
///  intial_pose(nuturtle_control::srv::InitialPose): Gives the new configuration for the robot.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <armadillo>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuslam/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;

/// \brief publishes the odometry messages and odometry transform
class Slam : public rclcpp::Node
{
public:
  /// \brief declares all the node parameters. publishers, subscribers, services and transforms
  Slam()
  : Node("slam")
  {
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("body_id", "unspecified");
    declare_parameter("odom_id", "green/odom"); 
    declare_parameter("wheel_left", "unspecified");
    declare_parameter("wheel_right", "unspecified");
    declare_parameter("sensor", true);
    wrad_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_ = get_parameter("track_width").get_parameter_value().get<double>();
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    bool sensor_ = get_parameter("sensor").get_parameter_value().get<bool>();

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
    

    // Initializing robot states
    internal_odom_ = turtlelib::DiffDrive{wrad_, track_};
    q_ = internal_odom_.get_config();
    qt = internal_odom_.get_config();
    qt_prev = internal_odom_.get_config(); // should be all zeros

    for(int i = 0; i < 2*n; i++){
      sys_covar.at(3+i, 3+i) = 999999999 * map_covar.at(i, i);
    }
    //  Initializing the Q matrix
    for(int i = 0; i < 3; i++){
      Q.at(i, i) = 0.1;
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Slam::jointstatesub_callback, this, std::placeholders::_1));

    if(sensor_== true){
      sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/fake_sensor", 10,
      std::bind(&Slam::sensorsub_callback, this, std::placeholders::_1));
    }
    else{
      sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/estimated", 10,
      std::bind(&Slam::sensorsub_callback, this, std::placeholders::_1));
    }

    tf_odom_green_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_map_odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    landmarks_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", 10);
    init_server_ =
      create_service<nuslam::srv::InitialPose>(
      "initial_pose",
      std::bind(&Slam::init_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    path_pub_ = create_publisher<nav_msgs::msg::Path>("green/nav_path", 10);
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
    tf_odom_green_->sendTransform(t);
    odom_pub_->publish(odom_);

    turtlelib::Transform2D T_or({q_.x, q_.y}, q_.theta);
    turtlelib::Transform2D T_mo = T_mr*T_or.inv();
    //publish broadcast from map to odom
    geometry_msgs::msg::TransformStamped tdash;
    tdash.header.stamp = get_clock()->now();
    tdash.header.frame_id = "map";
    tdash.child_frame_id = odom_id_;
    tdash.transform.translation.x = T_mo.translation().x;
    tdash.transform.translation.y = T_mo.translation().y;
    tf2::Quaternion thetadash;
    thetadash.setRPY(0, 0, T_mo.rotation());
    tdash.transform.rotation.x = thetadash.x();
    tdash.transform.rotation.y = thetadash.y();
    tdash.transform.rotation.z = thetadash.z();
    tdash.transform.rotation.w = thetadash.w();
    tf_map_odom_->sendTransform(tdash);


    path_.header.stamp = get_clock()->now();
    path_.header.frame_id = "map";
    pstmp_.pose.position.x = T_mr.translation().x;
    pstmp_.pose.position.y = T_mr.translation().y;
    tf2::Quaternion theta_r;
    theta_r.setRPY(0, 0, T_mr.rotation());
    pstmp_.pose.orientation.x = theta_r.x();
    pstmp_.pose.orientation.y = theta_r.y();
    pstmp_.pose.orientation.z = theta_r.z();
    pstmp_.pose.orientation.w = theta_r.w();
    path_.poses.push_back(pstmp_);
    path_pub_->publish(path_);
  }
  /// @brief Implements the EKF algorithm
  /// @param msg -The fake sensor obstacle markers
  void sensorsub_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    visualization_msgs::msg::MarkerArray landmarks;
    // call prediction
    prediction();
    // CORRECTION calculation
    for(long unsigned int i = 0; i < msg.markers.size(); i++){
      visualization_msgs::msg::Marker s_marker = msg.markers.at(i);
      // check if action is delete or not
      if (s_marker.action != visualization_msgs::msg::Marker::DELETE){
        // get the relative x and y position of the landmark
        double x_i = s_marker.pose.position.x, y_i = s_marker.pose.position.y;
        // calculate range bearing measurement
        double r_i = std::sqrt((x_i*x_i) + (y_i*y_i));
        double phi_i = turtlelib::normalize_angle(std::atan2(y_i, x_i));
        // RCLCPP_INFO_STREAM(get_logger(), "Before" << sys_state_p);
        int j = data_ass(r_i, phi_i);
        // calculate actual measurement
        arma::colvec zt_i=arma::colvec(2, arma::fill::zeros); //zt_i=arma::mat(2*n, 1, arma::fill::zeros);
        zt_i.at(0) = r_i;
        zt_i.at(1) = phi_i;
        // Calculate variables used for H matrix using predicted state
        double del_xi = sys_state_p(2*j+3) - sys_state_p(1);
        double del_yi = sys_state_p(2*j+4) - sys_state_p(2);
        double d_i = (del_xi*del_xi )+ (del_yi*del_yi);
        // calculate predicted measurements using predicted state
        double rcap_i = std::sqrt(d_i), phicap_i = turtlelib::normalize_angle(std::atan2(del_yi, del_xi) - sys_state_p.at(0));
        arma::colvec zcapt_i=arma::colvec(2, arma::fill::zeros); // zcapt_i=arma::mat(2*n, 1, arma::fill::zeros);
        zcapt_i(0) = rcap_i;
        zcapt_i(1) = phicap_i;
        // calculate H matrix
        arma::mat H_i=arma::mat(2, 3+2*n, arma::fill::zeros);
        H_i(0, 1) = -del_xi/rcap_i;
        H_i(0, 2) = -del_yi/rcap_i;
        H_i(1, 0) = -1.0;
        H_i(1, 1) = del_yi/d_i;
        H_i(1, 2) = -del_xi/d_i;
        H_i(0, 2*j+3) = del_xi/rcap_i;
        H_i(0, 2*j+4) = del_yi/rcap_i;
        H_i(1, 2*j+3) = -del_yi/d_i;
        H_i(1, 2*j+4) = del_xi/d_i;
        // calculate Kalman gain of this landmark
        arma::mat R_i=arma::mat(2, 2, arma::fill::eye); // COVARIANCE MATRIX
        R_i(0,0)=0.1; // 1.0;
        R_i(1,1)=0.1; //1.0;
        arma::mat temp = (H_i*sys_covar_p*H_i.t()) + R_i;
        arma::mat K_i = (sys_covar_p * H_i.t()) * ((H_i*sys_covar_p*H_i.t()) + R_i).i();
        // correct/update system state prediction
        arma::colvec difference = zt_i - zcapt_i;
        difference(1) = turtlelib::normalize_angle(difference(1));
        sys_state = sys_state_p + K_i*(difference);
        sys_state(0) = turtlelib::normalize_angle(sys_state(0));
        sys_state_p = sys_state;
        arma::mat I = arma::mat (3+2*n, 3+2*n, arma::fill::eye);
        sys_covar = (I - (K_i*H_i))*sys_covar_p;        
        sys_covar_p = sys_covar;
        // push landmark
        visualization_msgs::msg::Marker L_marker;
        L_marker.header.frame_id = "map";
        L_marker.header.stamp = get_clock()->now();
        L_marker.id = j; // can we keep track of markers with marker id
        L_marker.action = visualization_msgs::msg::Marker::ADD;
        L_marker.type = visualization_msgs::msg::Marker::CYLINDER;

        L_marker.pose.position.x = sys_state_p.at(3+2*j);
        L_marker.pose.position.y = sys_state_p.at(4+2*j);

        L_marker.pose.position.z = s_marker.pose.position.z;
        L_marker.pose.orientation = s_marker.pose.orientation;
        L_marker.scale = s_marker.scale;
        L_marker.color.r = 0.0;
        L_marker.color.g = 1.0;
        L_marker.color.b = 0.0;
        L_marker.color.a = 1.0;
        L_marker.frame_locked = false;
        landmarks.markers.push_back(L_marker);
      }
    }
    // publish landmarks
    landmarks_publisher_->publish(landmarks);
    T_mr = turtlelib::Transform2D ({sys_state_p.at(1), sys_state_p.at(2)}, sys_state_p.at(0));
  }

  /// @brief Calculated the predicted system state and covariance
  void prediction(){
    // prediction calculation
    // 1. Calculate change in robot state
    dq.x = q_.x - qt_prev.x;
    dq.y = q_.y - qt_prev.y;
    dq.theta = turtlelib::normalize_angle(q_.theta - qt_prev.theta);
    qt_prev = q_;
    // 2. Calculate A matrix
    A_t.at(1, 0) = -dq.y;
    A_t.at(2, 0) = dq.x;
    // Calculate system state prediction
    sys_state_p.at(0) = turtlelib::normalize_angle(sys_state_p(0)+ dq.theta);
    sys_state_p.at(1) = sys_state_p.at(1)+ dq.x;
    sys_state_p.at(2) = sys_state_p.at(2)+ dq.y;
    // Calculate covariance prediction
    sys_covar_p = (A_t*sys_covar*A_t.t()) + Q;

  }

  /// @brief Implements the data association algorithm
  /// @param r_i- The range bearing measurement of the marker
  ///@param phi_i-The angle of the measurement marker
  int data_ass(double r_i, double phi_i){
    arma::mat temp = sys_state_p;
    RCLCPP_INFO_STREAM(get_logger(), "COUNT" << count);
    int j = count;
    if(count == 0){
      sys_state_p(3)= sys_state_p(1)+ (r_i*cos(phi_i + sys_state_p(0)));
      sys_state_p(4)= sys_state_p(2) + (r_i*sin(phi_i + sys_state_p(0)));
      count ++;
      return j;
    }
    arma::colvec z_i =arma::colvec(2, arma::fill::zeros);
    z_i.at(0) = r_i;
    z_i.at(1) = phi_i;
    temp(2*j + 3) = temp(1)+ (r_i*cos(phi_i + temp(0)));
    temp(2*j + 4)= temp(2) + (r_i*sin(phi_i + temp(0)));
    arma::vec dist=arma::vec(j+1, arma::fill::zeros);
    for(int k=0; k<j+1; k++){
      double del_xk = temp(2*k+3) - temp(1);
      double del_yk = temp(2*k+4) - temp(2);
      double d_k = (del_xk*del_xk )+ (del_yk*del_yk);
      double rcap_k = std::sqrt(d_k), phicap_k = turtlelib::normalize_angle(std::atan2(del_yk, del_xk) - temp.at(0));
      arma::colvec z_k=arma::colvec(2, arma::fill::zeros);
      z_k(0) = rcap_k;
      z_k(1) = phicap_k;
      arma::mat H_k=arma::mat(2, 3+2*n, arma::fill::zeros);
      H_k(0, 1) = -del_xk/rcap_k;
      H_k(0, 2) = -del_yk/rcap_k;
      H_k(1, 0) = -1.0;
      H_k(1, 1) = del_yk/d_k;
      H_k(1, 2) = -del_xk/d_k;
      H_k(0, 2*k+3) = del_xk/rcap_k;
      H_k(0, 2*k+4) = del_yk/rcap_k;
      H_k(1, 2*k+3) = -del_yk/d_k;
      H_k(1, 2*k+4) = del_xk/d_k;
      arma::mat R_k=arma::mat(2, 2, arma::fill::eye);
      R_k(0,0)=0.1;
      R_k(1,1)=0.1;
      arma::mat psi = (H_k*sys_covar_p*H_k.t()) + R_k;
      arma::mat m_dist=(z_i-z_k).t()*inv(psi)*(z_i-z_k);
      double maha = m_dist(0,0);
      RCLCPP_INFO_STREAM(get_logger(), "maha" << maha);
      if(k==j){
        maha=thresh;
        }
      dist(k)=maha;
    }
    int dstar =arma::index_min(dist);
    if(dstar==j){
    sys_state_p(2*j+3)= sys_state_p(1)+ (r_i*cos(phi_i + sys_state_p(0)));
    sys_state_p(2*j+4)= sys_state_p(2) + (r_i*sin(phi_i + sys_state_p(0)));
    count ++;
    return j;
    }
    else{
    return dstar;
    }
  }


  /// @brief The location of the odometry is reset to requested configuration.
  /// @param request - This message has the x, y and theta values of the new configuration.
  void init_callback(
    nuslam::srv::InitialPose::Request::SharedPtr request,
    nuslam::srv::InitialPose::Response::SharedPtr)
  {

    internal_odom_ = turtlelib::DiffDrive{{request->theta, request->x, request->y},
      {0.0, 0.0}, wrad_, track_};
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_publisher_;
  rclcpp::Service<nuslam::srv::InitialPose>::SharedPtr init_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_odom_green_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_map_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  nav_msgs::msg::Odometry odom_;
  turtlelib::DiffDrive internal_odom_;
  double wrad_, track_, wleft_prev_ = 0.0, wright_prev_ = 0.0;
  turtlelib::Wheel_ang wang_;
  turtlelib::Config q_, qt, qt_prev, dq;
  turtlelib::Twist2D Vb_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pstmp_;
  int n = 50;
  arma::colvec sys_state_p=arma::vec(3+2*n, arma::fill::zeros), sys_state=arma::colvec(3+2*n, arma::fill::zeros);
  arma::mat m_t=arma::mat(2*n, 1, arma::fill::zeros), mt_prev=arma::mat(2*n, 1, arma::fill::zeros);
  arma::mat sys_covar_p=arma::mat(3+2*n, 3+2*n, arma::fill::zeros);
  arma::mat map_covar=arma::mat(2*n, 2*n, arma::fill::eye);
  arma::mat sys_covar=arma::mat(3+2*n, 3+2*n, arma::fill::zeros);
  arma::mat A_t=arma::mat(3+2*n, 3+2*n, arma::fill::eye), Q=arma::mat(3+2*n, 3+2*n, arma::fill::zeros);
  // visualization_msgs::msg::MarkerArray landmarks;
  // std::vector<int> landmark_id;
  // arma::mat zt=arma::mat(2*n, 1, arma::fill::zeros);
  turtlelib::Transform2D T_mr;
  int count = 0;
  double thresh =1.0; //0.1; //1.0;
};


/// @brief creates an nuslam node.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Slam>());
  } catch (std::runtime_error & err) {
    std::cerr << err.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}
