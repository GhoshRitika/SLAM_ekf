/// 
/// \file Slam
/// \brief This node detects landmarks using the 2D laser_scan data and publishes their locations
///  relative to the robots
///
///
/// PUBLISHERS:
///     estimated(visualization_msgs::msg::MarkerArray): Publishes the fitted landmark markers
///
/// SUBSCRIBERS:
///     sim_laser(visualization_msgs::msg::MarkerArray): Receives lidar scan data

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
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/circle_fitting.hpp"

using namespace std::chrono_literals;

/// \brief publishes the unsupervised learning circle regression landmarks
class Landmarks : public rclcpp::Node
{
public:
  /// \brief declares all the node parameters. publishers, subscribers, services and transforms
  Landmarks()
  : Node("landmarks")
  {
    thresh = 0.1;
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "sim_laser", 10,
      std::bind(&Landmarks::laser_sub_callback, this, std::placeholders::_1));
    estimate_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/estimated", 10);
  }
  private:

  double dist(turtlelib::Coord c1, turtlelib::Coord c2){
    return std::sqrt(((c1.x-c2.x)*(c1.x-c2.x))+((c1.y-c2.y)*(c1.y-c2.y)));
  }
  /// @brief Implements the unsupervised clustering algorithm
  /// @param msg - provides the ranges of the lidar scan
  void laser_sub_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    clusters.clear();
    std::vector<turtlelib::Coord> points;
    visualization_msgs::msg::MarkerArray landmarks;
    std::vector<turtlelib::Coord> centers;
    ang_inc_ = msg.angle_increment;
    for(int i = 0; i < 360; i++){
      double theta = i*ang_inc_;
      double x = cos(theta)*msg.ranges[i];
      double y = sin(theta)*msg.ranges[i];
      points.push_back({x,y});
    }
    for(long unsigned int i = 0; i < 360; i++){
      if(i==0){ //checking the first element
        clusters.push_back(std::vector<turtlelib::Coord>{points[0]});
      }
      else{
        if(dist(points[i], points[i-1])<thresh){ //CHECKING IF CLOSE TO PREVIOUS NEAREST NEIGHBOR
          for(long unsigned int j = 0; j<clusters.size(); j++){
            for(long unsigned int k=0; k<clusters[j].size(); k++){
              if(clusters[j][k].x==points[i-1].x && clusters[j][k].y==points[i-1].y){
                clusters[j].push_back(points[i]);
                break;
              }
            }
          }
        }
        else{
          clusters.push_back(std::vector<turtlelib::Coord>{points[i]});
        }
      }
      if(i==359){ //CHECKING IF ITS THE LAST ELEMENT
        if(dist(points[i], points[0])<thresh){
          int index1, index2;
          for(long unsigned int j = 0; j<clusters.size(); j++){
            for(long unsigned int k=0; k<clusters[j].size(); k++){
              if(clusters[j][k].x==points[0].x && clusters[j][k].y==points[0].y){
                index1 = j;
              }
              if(clusters[j][k].x==points[i].x && clusters[j][k].y==points[i].y){
                index2 = j;
              }
            }
          }
          if(index1!=index2){
            clusters[index1].insert(clusters[index1].end(), clusters[index2].begin(), clusters[index2].end());
            clusters[index2].clear();
          }
          }
        }
    }
    for (long unsigned int i = 0; i < clusters.size(); i++) {
      if(clusters[i].size()> 4){
        std::vector<double> circle = turtlelib::fit_circle(clusters[i]);
        if(circle[2]< 0.038+0.015 && circle[2]> 0.038-0.015){
          centers.push_back({circle[0],circle[1]});
        }
      }
    }
    for(int i = 0; i < (int)centers.size(); i++){
      for(int j = i+1; j < (int)centers.size(); j++){
        if(dist(centers[i],centers[j])<0.1){
          centers.erase(centers.begin()+j);
        }
      }
    }
    for (long unsigned int i=0; i<centers.size(); i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "/red/base_footprint";
      marker.header.stamp = get_clock()->now();
      marker.id = i;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.pose.position.x = centers[i].x;
      marker.pose.position.y = centers[i].y;
      marker.pose.position.z = 0.25 / 2.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * 0.038;
      marker.scale.y = 2.0 * 0.038;
      marker.scale.z = 0.25;
      marker.color.r = 0.0;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.frame_locked = false;
      landmarks.markers.push_back(marker);
    }
    estimate_publisher_->publish(landmarks);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_ ;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr estimate_publisher_;
  double ang_inc_, thresh;
  visualization_msgs::msg::MarkerArray landmarks;
  std::vector<std::vector<turtlelib::Coord>> clusters;
  std::vector<turtlelib::Coord> centers;
};

/// \brief creates a nusim node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
