/// \file Nusim
/// \brief This node controls the position of a robot, creates cylindrical obstacles and walls
///
/// PARAMETERS:
///     rate (double): This is the rate in Hz at which the timer_callback runs.
///     x0 (double): This is the inital or home x position of the turtle robot.
///     y0 (double): This is the inital or home y position of the turtle robot.
///     theta0 (double): This is the inital or home theta configuration of the turtle robot.
///     obstacle/x (std::vector<double>): This is the desired x position of the turtle robot.
///     obstacle/y (std::vector<double>): This is the desired y position of the turtle robot.
///     obstacle/theta (double): This is the desired theta configuration of the turtle robot.
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The distance between the wheels.
///     motor_cmd_max(int): The motors are provided commands in [-motor_cmd_max, motor_cmd_max].
///     motor_cmd_per_rad_sec(double): Each motor command "tick" is 0.024 rad/sec.
///     encoder_ticks_per_rad(double): The number of encoder "ticks" per radian.
///     collision_radius(double): This is some simplified geometry used for collision detection.
///     walls.x_length(double): The length of the arena in the world x direction.
///     walls.y_length(double): The length of the arena in the world y direction.
///     draw_only(bool): Determines if only the walls and obstacles are published.
///     input_noise(double): The noise of the wheel velocity.
///     slip_fraction(double): The slip applied to the wheels.
///     basic_sensor_variance(double): The noise in the relative obstacle positions.
///     max_range(double): The range of detecting obstacles.
///     angle_min(double): The minimum angle of laser.
///     angle_max(double): The maximum angle of laser.
///     angle_increment(double): The angular increment in laser readings.
///     time_increment(double): The time increment of laser.
///     scan_time(double): The scan time of a laser reading.
///     range_min(double): The minimum detectable distance an object from lidar.
///     range_max(double): The maximum detectable distance an object from lidar.
///     noise_level(double): The noise in the range calculations.
///     samples(int): The number of samples for which range is calculated.

/// PUBLISHERS:
///     timestep (std_msgs::msg::UInt64): Publishes the incrementing timestep value.
///     obstacles (visualization_msgs::msg::MarkerArray): Publishes an array of obstacle markers.
///     walls (visualization_msgs::msg::MarkerArray): Publishes an array of wall markers.
///     red/sensor_data (nuturtlebot_msgs::msg::SensorData): Publishes the wheel positions.
///     fake_sensor (visualization_msgs::msg::MarkerArray): Publishes obstacle markers wrt robot position.
///     red/nav_path (nav_msgs::msg::Path): Publishes the red robot path.
///     sim_laser (sensor_msgs::msg::LaserScan): Publishes the simulated lidar topic.

/// SUBSCRIBERS:
///     red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Receives wheel velocity in ticks.

/// SERVERS:
///     reset (std_srvs::srv::Empty): Rests the timestep to 0 and the robot to its initial position.
///     teleport (nusim::srv::Teleport): Gets the desired position of the robot.


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;


 std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }


/// \brief node controls the position of a robot and creates cylindrical obstacles
class Nusim : public rclcpp::Node
{
public:
  /// \brief declares all the node parameters, publishers, services, marker arrays and broadcasters
  Nusim()
  : Node("nusim"), timestep_(0), left_(0.0), right_(0.0), prev_right_(0.0), prev_left_(0.0)
  {
    declare_parameter("rate", 100.0); // was 200.0
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("walls.x_length", 10.0);
    declare_parameter("walls.y_length", 10.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.01);
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
    declare_parameter("collision_radius", -1.0);
    declare_parameter("draw_only", false);
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 1.0);
    declare_parameter("angle_min", 0.0);
    declare_parameter("angle_max", 6.25);
    declare_parameter("angle_increment", 0.02);
    declare_parameter("time_increment", 0.0005);
    declare_parameter("scan_time", 0.2);
    declare_parameter("range_min", 0.1);
    declare_parameter("range_max", 3.5);
    declare_parameter("noise_level", 0.01);
    declare_parameter("samples", 0);
    wrad_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_ = get_parameter("track_width").get_parameter_value().get<double>();
    cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    cmd_prs_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    etsr_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    crad_ = get_parameter("collision_radius").get_parameter_value().get<double>();
    ang_min_ = get_parameter("angle_min").get_parameter_value().get<double>();
    ang_max_ = get_parameter("angle_max").get_parameter_value().get<double>();
    ang_inc_ = get_parameter("angle_increment").get_parameter_value().get<double>();
    time_inc_ = get_parameter("time_increment").get_parameter_value().get<double>();
    scan_time_ = get_parameter("scan_time").get_parameter_value().get<double>();
    min_ = get_parameter("range_min").get_parameter_value().get<double>();
    max_ = get_parameter("range_max").get_parameter_value().get<double>();
    noise_level_ = get_parameter("noise_level").get_parameter_value().get<double>();
    samples_ = get_parameter("noise_level").get_parameter_value().get<double>();
    if (wrad_ < 0 || track_ < 0 || cmd_max_ < 0 || cmd_prs_ < 0 || etsr_ < 0 || crad_ < 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Did not define a parameter");
      throw std::runtime_error("ERROR LOADING PARAMETERS");
    }
    double rate_ = get_parameter("rate").get_parameter_value().get<double>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    wall_x = get_parameter("walls.x_length").get_parameter_value().get<double>();
    wall_y = get_parameter("walls.y_length").get_parameter_value().get<double>();
    // or use as_double_array();
    obstacles_x = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r = get_parameter("obstacles.r").get_parameter_value().get<double>();
    draw_only_ = get_parameter("draw_only").get_parameter_value().get<bool>();
    slip_fraction_ = get_parameter("slip_fraction").get_parameter_value().get<double>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<double>();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();
    double ht = 0.25;

    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_WARN(get_logger(), "******************KILLING NODE************************");
      throw std::runtime_error("Uneven array lengths");
    }

    robot_ = turtlelib::DiffDrive{wrad_, track_};
    phi.left = 0.0;
    phi.right = 0.0;
    q_ = robot_.get_config();
    long unsigned int i = 0;
    auto time_stamp = get_clock()->now();
    double wall_ht = 0.25;
    std::vector<double> wallx {(wall_x / 2) + 0.05, (-wall_x / 2) - 0.05, 0.0, 0.0};
    std::vector<double> wally {0.0, 0.0, (wall_y / 2) + 0.05, (-wall_y / 2) - 0.05};
    std::vector<double> scalex {0.1, 0.1, wall_x, wall_x};
    std::vector<double> scaley {wall_y, wall_y, 0.1, 0.1};
    // Setting the constant parameters of laser scan message
    laser_msg.header.frame_id = "red/base_scan";
    laser_msg.angle_min = ang_min_;
    laser_msg.angle_max = ang_max_;
    laser_msg.angle_increment = ang_inc_;

    laser_msg.range_min = min_;
    laser_msg.range_max = max_;

    for (i = 0; i < 4; i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "/nusim/world";
      marker.header.stamp = time_stamp;
      marker.id = i;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.pose.position.x = wallx.at(i);
      marker.pose.position.y = wally.at(i);
      marker.pose.position.z = wall_ht / 2.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = scalex.at(i);
      marker.scale.y = scaley.at(i);
      marker.scale.z = wall_ht;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.frame_locked = false;

      walls.markers.push_back(marker);
    }

    for (i = 0; i < size(obstacles_x); i++) {
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "/nusim/world";
      marker.header.stamp = time_stamp;
      marker.id = i;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.pose.position.x = obstacles_x.at(i);
      marker.pose.position.y = obstacles_y.at(i);
      marker.pose.position.z = ht / 2.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * obstacles_r;
      marker.scale.y = 2.0 * obstacles_r;
      marker.scale.z = ht;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.frame_locked = false;

      obstacles.markers.push_back(marker);
    }
    auto freq_ = 1s / rate_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    turtle_x_ = x0_;
    turtle_y_ = y0_;
    turtle_theta_ = theta0_;

    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    fake_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/fake_sensor", 10);
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    timer_ = create_wall_timer(freq_, std::bind(&Nusim::timer_callback, this));
    timer_5hz_ = create_wall_timer(0.2s, std::bind(&Nusim::timer_5hz_callback, this));
    wheelvel_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheelvel_callback, this, std::placeholders::_1));
    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("red/nav_path", 10);
    simLaser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("sim_laser", 10);
    reset_server_ =
      create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    teleport_server_ =
      create_service<nusim::srv::Teleport>(
      "teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  /// \brief Publishes the timestep, the marker arrays and the transform broadcast
  void timer_callback()
  {
    marker_publisher_->publish(obstacles);
    walls_publisher_->publish(walls);

    if(draw_only_== false){
      auto message = std_msgs::msg::UInt64();
      message.data = timestep_++;
      timestep_publisher_->publish(message);

      sensor_.stamp = get_clock()->now();
      std::uniform_real_distribution<> n(-slip_fraction_, slip_fraction_);

      n_r_ = n(get_random());
      n_l_ = n(get_random());
      double new_left = left_*0.01*(1.0 + n_l_);
      double new_right = right_*0.01*(1.0 + n_r_);
      robot_.FwdK({new_right, new_left});
      
      double updated_right = right_*0.01 + prev_right;
      double updated_left = left_*0.01 + prev_left;
      prev_left = updated_left;
      prev_right = updated_right;
      q_ = robot_.get_config();
      sensor_.left_encoder = static_cast<int>(updated_left * etsr_);
      sensor_.right_encoder = static_cast<int>(updated_right * etsr_);
      turtlelib::Vector2D robot =check_collision({q_.x, q_.y});

      q_.x = robot.x;
      q_.y = robot.y;
      robot_.set_config(q_);
      turtle_x_ = q_.x;
      turtle_y_ = q_.y;
      turtle_theta_ = q_.theta;

      sensor_pub_->publish(sensor_);
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "red/base_footprint";
      t.transform.translation.x = turtle_x_;
      
      t.transform.translation.y = turtle_y_;
      t.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, turtle_theta_);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);

      path_.header.stamp = get_clock()->now();
      path_.header.frame_id = "nusim/world";
      pstmp_.pose.position.x = q_.x;
      pstmp_.pose.position.y = q_.y;
      pstmp_.pose.orientation.x = q.x();
      pstmp_.pose.orientation.y = q.y();
      pstmp_.pose.orientation.z = q.z();
      pstmp_.pose.orientation.w = q.w();
      path_.poses.push_back(pstmp_);
      path_pub_->publish(path_);
    }
  }
  /// \brief Publishes the obstacles wrt robot position and calculated lidar information
  void timer_5hz_callback()
  {
    visualization_msgs::msg::MarkerArray fake_obs;
    std::normal_distribution<> n(0.0, std::sqrt(basic_sensor_variance_));
    double noise = n(get_random());
    turtlelib::Transform2D Twr({q_.x, q_.y}, q_.theta);
    turtlelib::Transform2D Trw = Twr.inv();
    long unsigned int i = 0;
    for (i = 0; i < obstacles_x.size(); i++) {
      turtlelib::Vector2D trans = Trw({obstacles_x.at(i), obstacles_y.at(i)});
      double d = std::sqrt((std::pow(trans.x, 2.0)) + (std::pow(trans.y, 2.0)));
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_footprint";
      marker.header.stamp = get_clock()->now();
      marker.id = i;
      if (d <= max_range_){
        marker.action = visualization_msgs::msg::Marker::ADD;
      }
      else{
        marker.action = visualization_msgs::msg::Marker::DELETE;

      }
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.pose.position.x = trans.x + noise;
      marker.pose.position.y = trans.y + noise;
      marker.pose.position.z = 0.25 / 2.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * obstacles_r;
      marker.scale.y = 2.0 * obstacles_r;
      marker.scale.z = 0.25;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.frame_locked = false;

      fake_obs.markers.push_back(marker);
    }
    fake_publisher_->publish(fake_obs);
    laser_msg.ranges.clear();
    set_laserscan();
    simLaser_pub_->publish(laser_msg);
  }

  /// \brief Resets the timestep and changes robot position to inital value
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    timestep_ = 0;
    turtle_x_ = x0_;
    turtle_y_ = y0_;
    turtle_theta_ = theta0_;
    robot_ = turtlelib::DiffDrive{wrad_, track_};
    q_ = robot_.get_config();

  }

  /// \brief Gets the desired position of the robot
  /// \param request - This message has the x, y and theta values of the new position
  void teleport_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    turtle_x_ = request->x;
    turtle_y_ = request->y;
    turtle_theta_ = request->theta;
  }

  /// \brief Receives the wheel velocities and converts them to rad/sec
  /// \param msg - This message has the right and left wheel velocities in ticks
  void wheelvel_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    left_ = static_cast<double>(msg.left_velocity) * cmd_prs_;
    right_ = static_cast<double>(msg.right_velocity) * cmd_prs_;
    // put checks so that noise and slip isnt added when velocities are 0
    std::normal_distribution<> d(0.0, std::sqrt(input_noise_));
    double noise = d(get_random());
    // adding zero mean gaussian noise
    if(left_!=0.0){
      left_ = left_ + noise;
    }
    if(right_!=0.0){
      right_ = right_ + noise;
    }
  }
  double dist(turtlelib::Vector2D p1, turtlelib::Vector2D p2){
    return std::sqrt(std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0));
  }
  turtlelib::Vector2D check_collision(turtlelib::Vector2D v_r){
    bool flag = false;
    long unsigned int i = 0;
    for (i = 0; i < size(obstacles_x); i++) {
      double distance = dist(v_r, {obstacles_x.at(i), obstacles_y.at(i)});
      if(distance <= crad_ + obstacles_r){
        flag = true;
        break;
      }
    }

    if(flag == true){
      turtlelib::Vector2D v_c{obstacles_x.at(i), obstacles_y.at(i)};
      turtlelib::Vector2D v = v_r - v_c;
      turtlelib::Vector2D norm_v = normalize(v);
      double d = crad_ + obstacles_r - dist(v_r, v_c);
      turtlelib::Vector2D v_n = d*norm_v + v_r;
      return v_n; 
    }

    return v_r;
  }
 /// \brief Calculated the simulated lidar data
  void set_laserscan(){
    auto R = robot_.get_config();
    laser_msg.header.stamp = get_clock()->now();
    std::normal_distribution<> n_d(0.0, noise_level_);
    double theta = 0.0;
    for(int i = 0; i < 360; i++){
      bool flag = false;
      theta = i*ang_inc_;
      double max_x = R.x + cos(theta + R.theta)*max_;
      double max_y = R.y + sin(theta + R.theta)*max_;
      double m = (max_y - R.y)/(max_x - R.x);
      std::vector<double> distances;
      for(long unsigned int j = 0; j < size(obstacles_x); j++){
        double u = R.y -m*R.x - obstacles_y.at(j);
        double a = 1+std::pow(m, 2.0), b= 2.0*(m*u - obstacles_x.at(j)), c = std::pow(obstacles_x.at(j), 2.0) + std::pow(u, 2.0)- std::pow(obstacles_r,2.0);
        double del = std::pow(b, 2.0) - 4.0*a*c;
        if(del == 0.0){
          //get 1 point
          double x = -b/(2.0*a);
          double y = m*(x - R.x) + R.y;
          if(std::signbit(x- R.x)== std::signbit(max_x- R.x) && std::signbit(y- R.y)== std::signbit(max_y- R.y)){
          distances.push_back(dist({x, y}, {R.x, R.y}));
            
            flag = true;
            }
        }
        else if(del > 0.0){
          //get 2 points, take the shortest distance
          double x1 = (-b+std::sqrt(del))/(2.0*a), x2 = (-b-std::sqrt(del))/(2.0*a);
          double y1 = m*(x1-R.x) +R.y, y2 = m*(x2-R.x) + R.y;
          if(std::signbit(x1 - R.x)== std::signbit(max_x - R.x) && std::signbit(y1 - R.y) == std::signbit(max_y - R.y)){
            double dist1 = dist(turtlelib::Vector2D{x1, y1}, turtlelib::Vector2D{R.x, R.y});
            double dist2 = dist(turtlelib::Vector2D{x2, y2}, turtlelib::Vector2D{R.x, R.y});
            distances.push_back((dist1<dist2) ? dist1 : dist2);
            flag = true;
          }
        }
      }
      if (flag == true){
        laser_msg.ranges.push_back(*min_element(distances.begin(), distances.end()));
        }
      else{
        //calculate wall stuff
        // the max point is inside the walls
        if(std::abs(max_x) < wall_x/2.0 && std::abs(max_y) < wall_y/2.0){
          laser_msg.ranges.push_back(dist(turtlelib::Vector2D {max_x, max_y}, turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
        }
        else if(max_x > 0.0 && max_y > 0.0){ // ++ quadrant
        //check distance between x=wall_x/2.0 and y=wall_y/2.0
          double x1 = wall_x/2.0, y2 = wall_y/2.0;
          double y1 = m*(x1-R.x) +R.y, x2 = ((y2-R.y)/m)+R.x;
          bool m1 = (std::signbit(x1 - R.x) == std::signbit(max_x - R.x) && std::signbit(y1 - R.y) == std::signbit(max_y - R.y));
          bool m2 = (std::signbit(x2 - R.x) == std::signbit(max_x - R.x) && std::signbit(y2 - R.y) == std::signbit(max_y - R.y));
          if (m1 && m2){
            double dist1 = dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y});
            double dist2 = dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y});
            laser_msg.ranges.push_back((dist1<dist2) ? dist1 + n_d(get_random()): dist2+ n_d(get_random()));
            
          }
          else if(m1){
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
          else {
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
        }
        else if(max_x > 0.0 && max_y < 0.0){ // +- quadrant
        //check distance between x=wall_x/2.0 and y=wall_y/2.0
          double x1 = wall_x/2.0, y2 = -wall_y/2.0;
          double y1 = m*(x1-R.x) + R.y, x2 = ((y2-R.y)/m)+R.x;
          bool m1 = (std::signbit(x1 - R.x) == std::signbit(max_x - R.x) && std::signbit(y1 - R.y) == std::signbit(max_y - R.y));
          bool m2 = (std::signbit(x2 - R.x) == std::signbit(max_x - R.x) && std::signbit(y2 - R.y) == std::signbit(max_y - R.y));
          if (m1 && m2 ){
            double dist1 = dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y});
            double dist2 = dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y});
            laser_msg.ranges.push_back((dist1<dist2) ? dist1+ n_d(get_random()) : dist2+ n_d(get_random()));
            
          }
          else if(m1){
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
          else {
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
        }
        else if(max_x < 0.0 && max_y >0.0){ // -+ quadrant
        //check distance between x=wall_x/2.0 and y=wall_y/2.0
          double x1 = -wall_x/2.0, y2 = wall_y/2.0;
          double y1 = m*(x1-R.x) +R.y, x2 = ((y2-R.y)/m)+R.x;
          bool m1 = (std::signbit(x1 - R.x) == std::signbit(max_x - R.x) && std::signbit(y1 - R.y) == std::signbit(max_y - R.y));
          bool m2 = (std::signbit(x2 - R.x) == std::signbit(max_x - R.x) && std::signbit(y2 - R.y) == std::signbit(max_y - R.y));
          if (m1 && m2){
            double dist1 = dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y});
            double dist2 = dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y});
            laser_msg.ranges.push_back((dist1<dist2) ? dist1 + n_d(get_random()): dist2+ n_d(get_random()));
          }
          else if(m1){
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
          else {
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
        }
        else if(max_x < 0.0 && max_y < 0.0){ // -- quadrant
        //check distance between x=wall_x/2.0 and y=wall_y/2.0
          double x1 = -wall_x/2.0, y2 = -wall_y/2.0;
          double y1 = m*(x1-R.x) +R.y, x2 = ((y2-R.y)/m)+R.x;
          bool m1 = (std::signbit(x1 - R.x) == std::signbit(max_x - R.x) && std::signbit(y1 - R.y) == std::signbit(max_y - R.y));
          bool m2 = (std::signbit(x2 - R.x) == std::signbit(max_x - R.x) && std::signbit(y2 - R.y) == std::signbit(max_y - R.y));
          if (m1 && m2){
            double dist1 = dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y});
            double dist2 = dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y});
            laser_msg.ranges.push_back((dist1<dist2) ? dist1 + n_d(get_random()): dist2+ n_d(get_random()));
          }
          else if(m1){
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x1, y1},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
          else {
            laser_msg.ranges.push_back(dist(turtlelib::Vector2D {x2, y2},turtlelib::Vector2D {R.x, R.y})+ n_d(get_random()));
          }
        }
        else{
          laser_msg.ranges.push_back((wall_x/2.0<wall_y/2.0) ? wall_x/2.0+ n_d(get_random()) : wall_y/2.0+ n_d(get_random()));
        }
      }
    }
  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_5hz_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelvel_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr simLaser_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  int timestep_;
  double turtle_x_, turtle_y_, turtle_theta_;
  double x0_, y0_, theta0_, input_noise_, slip_fraction_, n_r_, n_l_, basic_sensor_variance_, l, r, left, right;
  std::vector<double> obstacles_x, obstacles_y;
  double obstacles_r,  max_range_, wall_x, wall_y;
  int samples_;
  double wrad_, track_, cmd_prs_, etsr_, crad_;
  int cmd_max_;
  visualization_msgs::msg::MarkerArray obstacles;
  visualization_msgs::msg::MarkerArray walls;
  visualization_msgs::msg::MarkerArray fake_obs;
  double left_, right_, prev_right_, prev_left_;
  turtlelib::DiffDrive robot_;
  turtlelib::Config q_;
  nuturtlebot_msgs::msg::SensorData sensor_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pstmp_;
  bool draw_only_;
  turtlelib::Wheel_ang phi;
  double ang_min_ , ang_max_, ang_inc_,time_inc_, scan_time_, max_,min_, noise_level_;
  sensor_msgs::msg::LaserScan laser_msg;
  double prev_left = 0.0;
  double prev_right = 0.0;
};

/// \brief creates a nusim node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Nusim>());
  } catch (std::runtime_error & err) {
    std::cerr << err.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}
