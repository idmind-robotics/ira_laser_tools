#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ira_laser_tools
{

class LaserscanMerger : public rclcpp::Node
{
public:
  explicit LaserscanMerger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);
  void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 * merged_cloud);
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  void laserscan_topic_parser();
  void discoveryTimerCallback();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  laser_geometry::LaserProjection projector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;
  std::vector<bool> clouds_modified;

  std::vector<pcl::PCLPointCloud2> clouds;
  std::vector<std::string> input_topics;

  rclcpp::TimerBase::SharedPtr discovery_timer_;
  std::set<std::string> pending_topics_;

  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;
  double range_max;

  std::string destination_frame;
  std::string cloud_destination_topic;
  std::string scan_destination_topic;
  std::string laserscan_topics;
};

}  // namespace ira_laser_tools
