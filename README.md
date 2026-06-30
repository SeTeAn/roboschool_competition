# Roboschool Competition Controller

Autonomous controller implementation for an Aliengo robotics competition environment. The project combines RGB-D perception, YOLO-based object detection, object memory, path planning, and velocity control for a quadruped robot in simulation.

The base simulator, bridge, and competition environment were provided by the competition organizers. This repository focuses on the implemented controller logic: detection integration, navigation, and ROS2/Python controller variants.

## Project Scope

The competition task required the robot to work in a simulated environment, process camera and depth data, detect target objects, navigate toward relevant targets, and publish robot velocity commands.

This repository includes:

- a Python controller for the base competition runtime;
- a ROS2-oriented controller for bridge-based execution;
- an object detection integration module using an Ultralytics YOLO model artifact;
- navigation logic based on RGB-D perception, occupancy mapping, A* planning, Pure Pursuit control, and close-obstacle recovery.

## Contribution

The implemented project work in this repository is centered on:

- controller logic in `src/aliengo_competition/controllers/main_controller.py`;
- ROS2 controller adaptation in `ros2_isaac_bridge/sim_side/controller.py`;
- integration of YOLO inference into the perception pipeline;
- conversion of image detections and depth values into local object coordinates;
- occupancy grid updates from depth observations;
- object memory and mission target selection;
- A* path planning over the occupancy grid;
- Pure Pursuit command generation;
- close-obstacle recovery behavior.

The model was not trained as part of this work, and dataset labeling was outside the project scope. Model selection was handled in a team context.

## System Overview

```text
RGB-D camera data
        |
        v
Input handling and pose estimation
        |
        v
YOLO detection + depth-based localization
        |
        v
Object memory and mission logic
        |
        v
Occupancy grid + A* path planning
        |
        v
Pure Pursuit controller + obstacle recovery
        |
        v
Velocity commands for Aliengo
```

## Main Components

### Perception

The perception pipeline reads RGB and depth frames, runs YOLO inference, filters detections by confidence, and estimates local object coordinates using camera intrinsics and depth values.

Relevant code:

- `detect_markers(...)`
- `ScenePerception`
- `InputHandler`

### Detection Model

The controllers load the model artifact from:

```text
models/best.pt
```

The code uses the `ultralytics` package to load the YOLO model. The repository includes the model artifact used by the controller, but it does not include the training pipeline or dataset labeling workflow.

### Navigation

The navigation stack builds an internal map and selects movement commands toward active targets.

Relevant modules:

- `OccupancyGridMap`
- `ObjectMemory`
- `MissionLogic`
- `AStarPlanner`
- `PurePursuitController`
- `NavigationPlanner`
- `CloseObstacleRecovery`

### ROS2 Controller

The ROS2 controller subscribes to robot state and camera topics, runs the same high-level perception/navigation logic, and publishes velocity commands.

Main file:

```text
ros2_isaac_bridge/sim_side/controller.py
```

Key topic groups:

- `/cmd_vel`
- `/aliengo/base_velocity`
- `/aliengo/camera/color/image_raw`
- `/aliengo/camera/depth/image_raw`
- `/aliengo/joint_states`
- `/aliengo/imu`
- `competition/object_sequence`
- `competition/detected_object`

## Repository Structure

```text
.
|-- models/
|   `-- best.pt
|-- src/
|   `-- aliengo_competition/
|       `-- controllers/
|           `-- main_controller.py
|-- ros2_isaac_bridge/
|   |-- README.md
|   |-- run_bridge_node.sh
|   `-- sim_side/
|       |-- controller.py
|       |-- isaac_controller.py
|       `-- sim_bridge_client.py
|-- task-description.md
|-- setup.py
`-- README.md
```

## Technologies

- Python
- NumPy
- ROS2 / `rclpy`
- Ultralytics YOLO
- RGB-D camera processing
- Occupancy grid mapping
- A* path planning
- Pure Pursuit control
- Isaac Gym / Aliengo competition environment

## Running Notes

The original competition environment supports both Python-controller and ROS2 bridge-based workflows. See the existing environment files and `ros2_isaac_bridge/README.md` for infrastructure details.

The controller code expects the model artifact at:

```text
models/best.pt
```

Required Python-side dependencies include at least:

```text
numpy
ultralytics
```

The ROS2 controller additionally requires a ROS2 environment with the message packages used by the organizer-provided bridge.

## Limitations

- The simulator and base competition environment were provided by the organizers.
- The model was not trained in this repository.
- Dataset labeling was not part of this repository.
- No final competition score or benchmark metric is claimed here.
- The project is kept as a competition implementation artifact, not as a production robotics system.

## Original Task Context

The original competition task description is preserved in:

```text
task-description.md
```

That file describes the organizer-provided problem setting. This README focuses on the implemented controller and navigation/detection logic.
