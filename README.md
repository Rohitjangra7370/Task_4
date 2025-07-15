# ROS2 Autonomous Exploration Framework

## Project Synopsis

This repository encapsulates a sophisticated ROS2-based framework for autonomous robotic exploration, leveraging frontier-based algorithms and a bespoke A* global path planner integrated with the Nav2 navigation stack. Tailored for simulation environments utilizing TurtleBot3 in Gazebo, the system facilitates real-time mapping, obstacle avoidance, and efficient navigation in unknown terrains. Key packages include `explore` (a streamlined, frontier-centric exploration module ported to ROS2 Foxy), `auto_explo` (orchestrating SLAM and navigation launches), and `nav2_custom_planner` (a custom A* planner plugin optimized for exploratory tasks).

The architecture emphasizes modularity, enabling seamless integration with Nav2's layered costmaps, AMCL localization, and DWB local planning. This setup empowers robots to dynamically identify unexplored frontiers, compute optimal paths, and adapt to environmental uncertainties, making it an ideal substrate for robotics research, algorithmic prototyping, and educational simulations.

## Core Features

- **Frontier-Based Exploration Algorithm**: Employs a greedy frontier detection mechanism to prioritize unexplored regions. Frontiers are identified via breadth-first search on occupancy grids, filtering out invalid candidates based on cost thresholds and minimum size constraints.
- **Custom A* Global Planner**: A Nav2-compatible plugin implementing an enhanced A* heuristic search with diagonal movement support and cost-aware path optimization. Utilizes priority queues for efficient node expansion, incorporating Euclidean heuristics and obstacle inflation penalties.
- **SLAM Integration**: Harnesses the SLAM Toolbox for simultaneous localization and mapping, generating real-time occupancy grids with configurable parameters for laser scan processing and map updates.
- **Simulation Environment**: Pre-configured launches for TurtleBot3 in Gazebo, including RViz visualizations for costmaps, frontiers, and trajectories.
- **Parameter Tunability**: YAML-based configurations for fine-grained control over exploration dynamics, costmap layers (e.g., voxel and inflation), and planner tolerances.
- **Visualization Primitives**: Publishes marker arrays for frontier visualization in RViz, with color-coded representations of costs and centroids.
- **Blacklisting and Recovery Mechanisms**: Implements goal blacklisting to mitigate repeated failures in inaccessible areas, alongside progress timeouts for adaptive replanning.

## System Prerequisites

- **ROS2 Distribution**: Foxy or later (tested on Humble for extended compatibility).
- **Simulation Backend**: Gazebo Classic (version 11+ recommended for physics fidelity).
- **Navigation Stack**: Nav2 (navigation2 package) for core planning and control.
- **Robot Model**: TurtleBot3 packages (`turtlebot3_gazebo`, `turtlebot3_description`).
- **Dependencies**: SLAM Toolbox, TF2, visualization_msgs, and associated ROS2 libraries.

Install dependencies via:
```
sudo apt update
sudo apt install ros--navigation2 ros--slam-toolbox ros--turtlebot3*
rosdep install --from-paths src --ignore-src -r -y
```

## Installation Protocol

1. **Repository Cloning**:
   ```
   git clone https://github.com/rohitjangra7370/task_4.git
   cd task_4
   ```

2. **Workspace Compilation**:
   ```
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Dependency Resolution**:
   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Operational Workflow

### Simulation and Exploration Launch Sequence

1. **Initialize Gazebo Environment**:
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Activate SLAM for Mapping**:
   ```
   ros2 launch auto_explo slam.launch.py
   ```
   This spawns an asynchronous SLAM Toolbox node with configurable parameters (e.g., `mapper_params_localization.yaml` for odometry fusion).

3. **Engage Navigation and Exploration**:
   ```
   ros2 launch auto_explo navigation.launch.py
   ```
   Loads the custom A* planner via `nav2_param.yaml`, integrating AMCL for localization and DWB for local trajectory following.

4. **Standalone Exploration Node** (Optional):
   ```
   ros2 launch explore explore.launch.py
   ```
   Directly invokes the frontier search algorithm, publishing goals to Nav2.

RViz auto-launches with a predefined configuration (`navigation.rviz`) for monitoring costmaps, particle clouds, and frontier markers. The robot commences autonomous exploration, greedily selecting frontiers based on weighted costs (potential, gain, and orientation scales).

### Custom Planner Configuration

Register the A* plugin in `nav2_param.yaml` under the planner server:
```
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_custom_planner::CustomPlanner"
      use_astar: true  # Enable heuristic search
      allow_unknown: true  # Permit navigation through unknown space
      tolerance: 0.5  # Goal tolerance in meters
```

The planner computes paths using a priority queue, evaluating nodes with f(n) = g(n) + h(n), where h(n) is the Euclidean distance heuristic, supporting 8-connected neighborhoods for smoother trajectories.

### Map Persistence

Post-exploration, serialize the generated map:
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Configuration Schema

All parameters are YAML-configurable for runtime adaptability.

- **Exploration Parameters** (`explore/config/params.yaml`):
  - `planner_frequency`: Frontier computation rate (Hz, default: 0.15).
  - `min_frontier_size`: Threshold for frontier validity (meters, default: 0.75).
  - `visualize`: Toggle frontier marker publication (default: true).
  - `potential_scale`: Weight for distance-based frontier potential (default: 3.0).
  - `gain_scale`: Weight for frontier size gain (default: 1.0).

- **Navigation Parameters** (`auto_explo/config/nav2_param.yaml`):
  - Costmap plugins: Voxel layer for 3D obstacle marking, inflation for safety buffers.
  - AMCL: Particle filter with likelihood field model (e.g., `sigma_hit: 0.2`).
  - DWB Local Planner: Velocity/acceleration limits tuned for TurtleBot3 (e.g., `max_vel_x: 0.26`).
  - Behavior Tree: Custom recovery behaviors like spin and backup.

- **Map Artifacts**: Static map in `auto_explo/config/my_map_slam.yaml` for bootstrapped navigation, with trinary mode and occupancy thresholds.

Refer to `explore/doc/architecture.dia` for a diagrammatic overview of data flows and module interactions.

## Architectural Blueprint

The framework orchestrates:
- **SLAM Toolbox**: Asynchronous mapping with occupancy grid updates via `async_slam_toolbox_node`.
- **Nav2 Stack**: Modular planning pipeline with global A* and local DWB controllers.
- **Frontier Exploration**: BFS-based search on costmaps, computing centroids and costs for goal selection.
- **Custom Integrations**: TF2 for transform management, ensuring frame consistency between `map`, `odom`, and `base_link`.

For in-depth architecture, consult `explore/doc/architecture.dia` and `wiki_doc.txt`.

## Contribution Guidelines

Fork the repository and adhere to:
1. Branching: `git checkout -b feature/`.
2. Commit: `git commit -am 'Implement '`.
3. Push: `git push origin feature/`.
4. Pull Request: Submit via GitHub with detailed changelogs.

Issues and enhancements can be reported through GitHub Issues.

## Licensing

Licensed under BSD-3-Clause. Package-specific licenses (e.g., `explore` under BSD) are detailed in respective manifests.

For inquiries, contact the maintainer at rohitjangra7370@gmail.com.
