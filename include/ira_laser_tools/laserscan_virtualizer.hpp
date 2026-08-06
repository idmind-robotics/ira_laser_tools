#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/PCLHeader.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ira_laser_tools
{

class LaserscanVirtualizer : public rclcpp::Node
{
public:
  explicit LaserscanVirtualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index);
  void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr pcl_in);
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  void virtual_laser_scan_parser();
  void discoveryTimerCallback();

  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::vector<tf2::Stamped<tf2::Transform>> transform_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> virtual_scan_publishers;
  std::vector<std::string> output_frames;

  rclcpp::TimerBase::SharedPtr discovery_timer_;
  std::set<std::string> pending_frames_;

  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;
  double range_max;

  std::string cloud_frame;
  std::string base_frame;
  std::string cloud_topic;
  std::string output_laser_topic;
  std::string virtual_laser_scan;
};

}  // namespace ira_laser_tools
