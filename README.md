# Obstacle Detector (ROS 2)

This package is a **ROS 2 port** of the [tysik/obstacle_detector](https://github.com/tysik/obstacle_detector) package.
Special thanks to **Mateusz Tyszkiewicz (tysik)** for the original implementation.

Currently, the `obstacle_extractor` node has been fully ported to ROS 2, and a new `obstacle_visualizer` node has been added for RViz visualization.

## Features

- **Obstacle Extraction**: Converts 2D LaserScan data into geometric primitives (Segments and Circles).
- **Visualization**: Visualizes detected obstacles in RViz using Markers.
- **Dynamic Reconfigure**: Supports real-time parameter tuning via `rqt_reconfigure`.
- **Front-Only Mode**: Option to process only the front half of the LiDAR data.

## Dependencies

- **ROS 2** (Tested on Jazzy, compatible with Humble/Iron)
- **Armadillo** (C++ Linear Algebra Library)
  ```bash
  sudo apt install libarmadillo-dev
  ```

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Josungjun/obstacles_detector.git obstacle_detector
cd ~/ros2_ws
colcon build --symlink-install --packages-select obstacle_detector
source install/setup.bash
```

## Usage

### 1. Run Obstacle Extractor
This node subscribes to `/scan` and publishes `/raw_obstacles`.

```bash
ros2 launch obstacle_detector obstacle_extractor.launch.py
```

### 2. Run Visualizer
This node converts `/raw_obstacles` into `/obstacle_markers` for RViz.

```bash
ros2 launch obstacle_detector obstacle_visualizer.launch.py
```

### 3. Visualization in RViz
1. Open RViz2: `rviz2`
2. Add **MarkerArray** display.
3. Set Topic to `/obstacle_markers`.
4. Set **Fixed Frame** to your LiDAR frame (e.g., `map`, `laser`, or `base_link`).

## Parameters

You can modify parameters in `config/params.yaml` or tune them dynamically using `rqt`.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `active` | `true` | Activate/Deactivate the node. |
| `use_scan` | `true` | Use 2D LaserScan messages. |
| `use_pcl` | `false` | Use PointCloud messages. |
| `use_front_half_only` | `false` | **(New)** If true, only process data from the front 180° (-90° to +90°). |
| `min_group_points` | `5` | Minimum number of points to form a group. |
| `max_group_distance` | `0.1` | Maximum distance between points to be in the same group. |
| `max_circle_radius` | `0.6` | Maximum radius for a circle. Objects larger than this are treated as segments (walls). |
| `circles_from_visibles` | `true` | Detect circles from visible segments. |

## Nodes

### `obstacle_extractor`
- **Subscribes**: `/scan` (sensor_msgs/LaserScan) or `/pcl` (sensor_msgs/PointCloud)
- **Publishes**: `/raw_obstacles` (obstacle_detector/Obstacles)

### `obstacle_visualizer`
- **Subscribes**: `/raw_obstacles`
- **Publishes**: `/obstacle_markers` (visualization_msgs/MarkerArray)
