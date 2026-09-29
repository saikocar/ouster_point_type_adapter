# Ouster PointCloud Adapter

This package provides a ROS2 composable node for converting a `PointCloud2` message from Ouster Format to Autoware Format.

## Features

- Subscribes to `PointCloud2` messages in [Ouster Format](https://github.com/ouster-lidar/ouster-ros/blob/master/include/ouster_ros/os_point.h).
- Publishes `PointCloud2` messages in [Autoware Format](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/).

## Changelog
- Added ability to limit intensity range (seemingly helps with Centerpoint if you keep intensity range within Velodyne values)
- Added ability to switch intensity between Reflectivity or Intensity channels
- Properly handle timestamps (copy timestamp from original message)

## Usage

1. Build the package:
   ```bash
   colcon build --symlink-install --packages-select ouster_point_type_adapter
2. Adapt topic remappings in `ouster_point_type_adapter.launch.py` as needed.
3. Launch `ouster_point_type_adapter` component in a new container:
   ```bash
   ros2 launch ouster_point_type_adapter ouster_point_type_adapter.launch.py

## sort_by_time (2026-09-29)
- Parameter `sort_by_time` (bool, default `true`): publish points sorted by `time_stamp` (stable sort, ring order kept within a column).
- Why: ouster-ros fills the cloud row-major (ring by ring), so timestamps jump back at every ring boundary.
  Autoware's distortion corrector integrates ego motion sequentially from the previously iterated point,
  which leaves a lateral offset proportional to lateral acceleration (about 0.3-0.4 m per 1 m/s^2 for 128 rings),
  and it takes the first point in the array as the reference time.
- The node prints `version <branch>@<hash>` at startup.

## stamp_at_first_point (2026-09-29)
- Parameter `stamp_at_first_point` (bool, default `false`): set the output `header.stamp` to the time of the earliest point instead of the scan start.
- Why: Autoware's distortion corrector aligns all points to the pose at the first point in the array but keeps the input header.
  With an `azimuth_window` that drops the first columns, the cloud is shifted along the travel direction by speed x (first point - header)
  (Ergamio: front_right 27.8 ms, front_left 8.4 ms; right vs top +0.22 m above 32 km/h). Concatenate compensates header differences with twist.
- Use together with `sort_by_time: true`.
