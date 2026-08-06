# ira_laser_tools

Updated with changes to ROS 2 Humble from https://github.com/nakai-omer/ira_laser_tools/tree/humble

Utilities for handling `sensor_msgs/msg/LaserScan` data: merging multiple laser scans into one,
and generating virtual laser scans from a point cloud. Both nodes are available as standalone
executables and as `rclcpp_components` composable nodes, and both expose their parameters as
dynamically reconfigurable (`add_on_set_parameters_callback`).

## laserscan_multi_merger

Merges several same-plane `LaserScan` topics into a single `LaserScan` (and the intermediate
merged `PointCloud2`), for use with algorithms that expect a single scan input (e.g. `gmapping`,
`amcl`). Topics to merge are discovered by name against the ROS graph at startup (and retried
every second until all are found), so producers may start after the merger.

The resulting scan appears generated from a single virtual scanner, disregarding actual
occlusions between the merged scans: for example, two scanners mounted on the front corners A
and B of a rectangular vehicle, each covering 270°, merge into a scan that looks as if it came
from a single scanner positioned halfway between A and B, regardless of what that virtual
position would actually be able to see.

### Usage

```bash
ros2 launch ira_laser_tools merge_multi.launch.py
```

Loaded as a composable node in a container (for zero-copy intra-process transport with other
components, e.g. a camera driver publishing point clouds):

```bash
ros2 launch ira_laser_tools merge_multi_composed.launch.py
```

Or standalone:

```bash
ros2 run ira_laser_tools laserscan_multi_merger --ros-args --params-file config/laserscan_merge.yaml
```

### API

| Topic | Type | Direction | Description |
|---|---|---|---|
| (topics listed in `laserscan_topics`) | `sensor_msgs/msg/LaserScan` | subscribed | Input scans to merge |
| `cloud_destination_topic` | `sensor_msgs/msg/PointCloud2` | published | Merged point cloud, in `destination_frame` |
| `scan_destination_topic` | `sensor_msgs/msg/LaserScan` | published | Merged scan, reprojected from the merged cloud |

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `destination_frame` | string | `base_link` | Frame all input scans are transformed into before merging |
| `cloud_destination_topic` | string | `/merged_cloud` | Output point cloud topic |
| `scan_destination_topic` | string | `/scan_multi` | Output merged scan topic |
| `laserscan_topics` | string | `""` | Space-separated list of input `LaserScan` topic names |
| `angle_min` | double | `-3.14` | Minimum angle of the output scan (rad) |
| `angle_max` | double | `3.14` | Maximum angle of the output scan (rad) |
| `angle_increment` | double | `0.0058` | Angular resolution of the output scan (rad) |
| `scan_time` | double | `0.0` | `scan_time` field of the output scan |
| `range_min` | double | `0.0` | Minimum accepted range (m) |
| `range_max` | double | `25.0` | Maximum accepted range (m); also used as the max range when projecting each input scan into a point cloud |

## laserscan_virtualizer

Generates one or more virtual `LaserScan` topics from a `PointCloud2` (e.g. one produced by a
multi-plane scanner such as a Velodyne), one per output frame listed in `virtual_laser_scan`, by
extracting the points close to each frame's z=0 plane. The roto-translation between each virtual
scanner frame and the base frame must be available on TF. Frames are resolved against TF at
startup and retried every second until available.

### Usage

```bash
ros2 run ira_laser_tools laserscan_virtualizer --ros-args --params-file <your_config>.yaml
```

Composable node plugin: `ira_laser_tools::LaserscanVirtualizer` (see `merge_multi_composed.launch.py`
for the container pattern; swap the `ComposableNode` for this plugin and its parameters).

### API

| Topic | Type | Direction | Description |
|---|---|---|---|
| `cloud_topic` | `sensor_msgs/msg/PointCloud2` | subscribed | Input point cloud |
| (one per frame in `virtual_laser_scan`, or `output_laser_topic` if set) | `sensor_msgs/msg/LaserScan` | published | One virtual scan per resolved frame |

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `base_frame` | string | `base_link` | Frame each virtual scan frame must be transformable to/from |
| `cloud_topic` | string | `/cloud_pcd` | Input point cloud topic |
| `output_laser_topic` | string | `/scan` | If non-empty, all virtual scans publish on this single topic; if empty, each publishes on a topic named after its frame |
| `virtual_laser_scan` | string | `"scansx scandx"` | Space-separated list of TF frame names to generate virtual scans for |
| `angle_min` | double | `-3.14` | Minimum angle of each virtual scan (rad) |
| `angle_max` | double | `3.14` | Maximum angle of each virtual scan (rad) |
| `angle_increment` | double | `0.0058` | Angular resolution of each virtual scan (rad) |
| `scan_time` | double | `0.0` | `scan_time` field of each virtual scan |
| `range_min` | double | `0.0` | Minimum accepted range (m) |
| `range_max` | double | `25.0` | Maximum accepted range (m) |

## Development

Core projection logic (`ira_laser_tools::projectPointsToScan`, in `include/ira_laser_tools/scan_projection.hpp`)
is ROS-independent and unit tested (`test/test_scan_projection.cpp`). Composition is covered by
a `launch_testing` integration test (`test/test_composition.py`). Run both with:

```bash
colcon test --packages-select ira_laser_tools
colcon test-result --verbose
```

Paper link: https://arxiv.org/abs/1411.1086

For questions, contact furlan@disco.unimib.it or augusto.ballardini@unimib.it (original authors).
