import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def make_scan(frame_id):
    scan = LaserScan()
    scan.header.frame_id = frame_id
    scan.angle_min = -1.0
    scan.angle_max = 1.0
    scan.angle_increment = 0.1
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = [5.0] * 21
    return scan


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    container = ComposableNodeContainer(
        name="test_ira_laser_tools_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ira_laser_tools",
                plugin="ira_laser_tools::LaserscanMerger",
                name="laserscan_multi_merger",
                parameters=[
                    {
                        "destination_frame": "base_link",
                        "laserscan_topics": "/scan_a /scan_b",
                        "angle_min": -1.0,
                        "angle_max": 1.0,
                        "angle_increment": 0.1,
                        "range_min": 0.0,
                        "range_max": 10.0,
                    }
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([
        container,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMergerComposition(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_merger_composition_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_merged_scan_is_published(self):
        pub_a = self.node.create_publisher(LaserScan, "/scan_a", qos_profile_sensor_data)
        pub_b = self.node.create_publisher(LaserScan, "/scan_b", qos_profile_sensor_data)

        received = []
        self.node.create_subscription(
            LaserScan, "/scan_multi", lambda msg: received.append(msg), qos_profile_sensor_data)

        end_time = self.node.get_clock().now().nanoseconds + 20_000_000_000
        while not received and self.node.get_clock().now().nanoseconds < end_time:
            pub_a.publish(make_scan("base_link"))
            pub_b.publish(make_scan("base_link"))
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(received, "No message received on /scan_multi from the composed node")
