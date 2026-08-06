#include "ira_laser_tools/scan_projection.hpp"

#include <cmath>

namespace ira_laser_tools
{

void projectPointsToScan(const Eigen::MatrixXf & points, sensor_msgs::msg::LaserScan & output)
{
  const double range_min_sq = output.range_min * output.range_min;

  for (int i = 0; i < points.cols(); i++) {
    const float & x = points(0, i);
    const float & y = points(1, i);
    const float & z = points(2, i);

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      continue;
    }

    const double range_sq = x * x + y * y;
    if (range_sq < range_min_sq) {
      continue;
    }

    const double angle = std::atan2(y, x);
    if (angle < output.angle_min || angle > output.angle_max) {
      continue;
    }

    const int index = static_cast<int>((angle - output.angle_min) / output.angle_increment);
    if (index < 0 || static_cast<std::size_t>(index) >= output.ranges.size()) {
      continue;
    }

    if (output.ranges[index] * output.ranges[index] > range_sq) {
      output.ranges[index] = std::sqrt(range_sq);
    }
  }
}

}  // namespace ira_laser_tools
