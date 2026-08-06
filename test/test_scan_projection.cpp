#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "ira_laser_tools/scan_projection.hpp"

namespace
{

sensor_msgs::msg::LaserScan makeScan(double angle_min, double angle_max, double angle_increment, double range_min)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = angle_min;
  scan.angle_max = angle_max;
  scan.angle_increment = angle_increment;
  scan.range_min = range_min;
  const auto ranges_size = static_cast<std::size_t>(std::ceil((angle_max - angle_min) / angle_increment));
  scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  return scan;
}

}  // namespace

TEST(ScanProjection, PointAtExactAngleMaxDoesNotWriteOutOfBounds)
{
  // angle_max sits exactly on a bin's angle, historically the out-of-bounds write.
  auto scan = makeScan(-3.14, 3.14, 0.00174, 0.1);
  Eigen::MatrixXf points(3, 1);
  const float angle = static_cast<float>(scan.angle_max);
  points.col(0) << std::cos(angle), std::sin(angle), 0.0f;

  EXPECT_NO_FATAL_FAILURE(ira_laser_tools::projectPointsToScan(points, scan));
}

TEST(ScanProjection, NanPointIsRejected)
{
  auto scan = makeScan(-1.0, 1.0, 0.1, 0.1);
  Eigen::MatrixXf points(3, 1);
  points.col(0) << std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f;

  ira_laser_tools::projectPointsToScan(points, scan);

  for (const auto & range : scan.ranges) {
    EXPECT_TRUE(std::isinf(range));
  }
}

TEST(ScanProjection, PointBelowRangeMinIsRejected)
{
  auto scan = makeScan(-1.0, 1.0, 0.1, 1.0);
  Eigen::MatrixXf points(3, 1);
  points.col(0) << 0.1f, 0.0f, 0.0f;  // range = 0.1, below range_min = 1.0

  ira_laser_tools::projectPointsToScan(points, scan);

  for (const auto & range : scan.ranges) {
    EXPECT_TRUE(std::isinf(range));
  }
}

TEST(ScanProjection, NearestReturnWinsWhenTwoPointsShareABin)
{
  auto scan = makeScan(-1.0, 1.0, 0.1, 0.0);
  Eigen::MatrixXf points(3, 2);
  points.col(0) << 5.0f, 0.0f, 0.0f;
  points.col(1) << 2.0f, 0.0f, 0.0f;

  ira_laser_tools::projectPointsToScan(points, scan);

  const int index = static_cast<int>((0.0 - scan.angle_min) / scan.angle_increment);
  EXPECT_NEAR(scan.ranges[index], 2.0, 1e-3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
