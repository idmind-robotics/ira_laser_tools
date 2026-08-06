#include "ira_laser_tools/laserscan_multi_merger.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

#include "ira_laser_tools/scan_projection.hpp"
#include "pcl_ros/transforms.hpp"

using namespace std;
using namespace pcl;

using std::placeholders::_1;

namespace ira_laser_tools
{

LaserscanMerger::LaserscanMerger(const rclcpp::NodeOptions & options)
: Node("laserscan_multi_merger", options)
{
	this->declare_parameter<std::string>("destination_frame", "base_link");
	this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
	this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
	this->declare_parameter<std::string>("laserscan_topics", "");
	this->declare_parameter("angle_min", -3.14);
	this->declare_parameter("angle_max", 3.14);
	this->declare_parameter("angle_increment", 0.0058);
	this->declare_parameter("scan_time", 0.0);
	this->declare_parameter("range_min", 0.0);
	this->declare_parameter("range_max", 25.0);

	this->get_parameter("destination_frame", destination_frame);
	this->get_parameter("cloud_destination_topic", cloud_destination_topic);
	this->get_parameter("scan_destination_topic", scan_destination_topic);
	this->get_parameter("laserscan_topics", laserscan_topics);
	this->get_parameter("angle_min", angle_min);
	this->get_parameter("angle_max", angle_max);
	this->get_parameter("angle_increment", angle_increment);
	this->get_parameter("scan_time", scan_time);
	this->get_parameter("range_min", range_min);
	this->get_parameter("range_max", range_max);

	dyn_params_handler_ = this->add_on_set_parameters_callback(
			std::bind(&LaserscanMerger::reconfigureCallback, this, _1));

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic.c_str(), rclcpp::SensorDataQoS());
	laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic.c_str(), rclcpp::SensorDataQoS());

	istringstream iss(laserscan_topics);
	pending_topics_ = std::set<std::string>(
			istream_iterator<string>(iss), istream_iterator<string>());

	discovery_timer_ = this->create_wall_timer(
			std::chrono::seconds(1), std::bind(&LaserscanMerger::discoveryTimerCallback, this));
	discoveryTimerCallback();
}

rcl_interfaces::msg::SetParametersResult LaserscanMerger::reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;

	for (auto parameter : parameters)
	{
		const auto &type = parameter.get_type();
		const auto &name = parameter.get_name();

		// Make sure it is a double value
		if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
		{
			double value = parameter.as_double();

			if (name == "angle_min")
			{
				this->angle_min = value;
			}
			else if (name == "angle_max")
			{
				this->angle_max = value;
			}
			else if (name == "angle_increment")
			{
				this->angle_increment = value;
			}
			else if (name == "time_increment")
			{
				this->time_increment = value;
			}
			else if (name == "scan_time")
			{
				this->scan_time = value;
			}
			else if (name == "range_min")
			{
				this->range_min = value;
			}
			else if (name == "range_max")
			{
				this->range_max = value;
			}
		}
	}

	result.successful = true;
	return result;
}

void LaserscanMerger::discoveryTimerCallback()
{
	if (pending_topics_.empty())
	{
		discovery_timer_->cancel();
		return;
	}

	laserscan_topic_parser();

	if (!pending_topics_.empty())
	{
		std::ostringstream missing;
		std::copy(pending_topics_.begin(), pending_topics_.end(), std::ostream_iterator<std::string>(missing, " "));
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for topics: %s", missing.str().c_str());
	}
	else
	{
		discovery_timer_->cancel();
	}
}

void LaserscanMerger::laserscan_topic_parser()
{
	// LaserScan topics to subscribe
	std::map<std::string, std::vector<std::string>> topics = this->get_topic_names_and_types();

	std::vector<string> tmp_input_topics;

	for (const auto &topic_it : topics)
	{
		std::vector<std::string> topic_types = topic_it.second;

		if (std::find(topic_types.begin(), topic_types.end(), "sensor_msgs/msg/LaserScan") != topic_types.end() && pending_topics_.erase(topic_it.first) > 0)
		{
			tmp_input_topics.push_back(topic_it.first);
		}
	}

	if (tmp_input_topics.empty())
	{
		return;
	}

	for (const auto &topic : input_topics)
	{
		tmp_input_topics.push_back(topic);
	}

	sort(tmp_input_topics.begin(), tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{
		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
			RCLCPP_INFO(this->get_logger(), "Subscribing to topics\t%ld", scan_subscribers.size());
			for (std::vector<int>::size_type i = 0; i < input_topics.size(); ++i)
			{
				// workaround for std::bind https://github.com/ros2/rclcpp/issues/583
				std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback =
						std::bind(
								&LaserscanMerger::scanCallback,
								this, std::placeholders::_1, input_topics[i]);
				scan_subscribers[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(input_topics[i].c_str(), rclcpp::SensorDataQoS(), callback);
				clouds_modified[i] = false;
				RCLCPP_INFO(this->get_logger(), "\t%s", input_topics[i].c_str());
			}
		}
	}
}

void LaserscanMerger::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic)
{
	sensor_msgs::msg::PointCloud2 tmpCloud1, tmpCloud2;

	try
	{
		// Verify that TF knows how to transform from the received scan to the destination scan frame
		tf_buffer_->lookupTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, rclcpp::Duration(1, 0));
		projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, *tf_buffer_, range_max);
		pcl_ros::transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2, *tf_buffer_);
	}
	catch (tf2::TransformException &ex)
	{
		return;
	}

	for (std::vector<int>::size_type i = 0; i < input_topics.size(); i++)
	{
		if (topic.compare(input_topics[i]) == 0)
		{
			pcl_conversions::toPCL(tmpCloud2, clouds[i]);
			clouds_modified[i] = true;
		}
	}

	// Count how many scans we have
	std::vector<int>::size_type totalClouds = 0;
	for (std::vector<int>::size_type i = 0; i < clouds_modified.size(); i++)
	{
		if (clouds_modified[i])
		{
			totalClouds++;
		}
	}

	// Go ahead only if all subscribed scans have arrived
	if (totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for (std::vector<int>::size_type i = 1; i < clouds_modified.size(); i++)
		{
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
			merged_cloud += clouds[i];
#else
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
#endif

			clouds_modified[i] = false;
		}

		Eigen::MatrixXf points;

		pcl::getPointCloudAsEigen(merged_cloud, points);

		pointcloud_to_laserscan(points, &merged_cloud);

		if (point_cloud_publisher_->get_subscription_count() > 0)
		{
			// Publish point cloud after publishing laser scan as for some reason moveFromPCL is causing getPointCloudAsEigen to
			// throw a segmentation fault crash
			auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

			pcl_conversions::moveFromPCL(merged_cloud, *cloud_msg);

			point_cloud_publisher_->publish(std::move(cloud_msg));
		}
	}
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	auto output = std::make_unique<sensor_msgs::msg::LaserScan>();
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

	ira_laser_tools::projectPointsToScan(points, *output);

	laser_scan_publisher_->publish(std::move(output));
}

}  // namespace ira_laser_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ira_laser_tools::LaserscanMerger)
