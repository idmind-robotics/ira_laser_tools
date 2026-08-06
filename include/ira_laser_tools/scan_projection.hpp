#pragma once

#include <Eigen/Dense>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace ira_laser_tools
{

// Projects a Nx3+ point matrix onto `output.ranges`, keeping the nearest return per angular
// bin. `output` must already have angle_min/angle_max/angle_increment set and `ranges` sized
// and filled with the caller's default value (e.g. infinity or range_max + 1.0). Points that
// are NaN, below range_min, outside [angle_min, angle_max], or that fall past the last bin due
// to floating point rounding are silently skipped.
void projectPointsToScan(const Eigen::MatrixXf & points, sensor_msgs::msg::LaserScan & output);

}  // namespace ira_laser_tools
