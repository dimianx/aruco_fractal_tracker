# ArUco Fractal Tracker

This ROS2 package is designed to detect and estimate the pose of ArUco fractal markers.

![Example Detection](example_marker/example_detection.gif)

## Features
- Detects ArUco fractal markers in images.
- Estimates the pose of detected markers with respect to a camera frame.
- Publishes the marker pose as a `geometry_msgs::msg::PoseStamped`.
- Broadcasts the transform from the camera frame to the marker frame using TF.

## Installation

### Prerequisites

- **ROS2**: Make sure you have ROS2 installed on your system. This package has been tested with ROS2 Foxy, but it should work with other ROS2 distributions as well.
- **OpenCV**: For image processing.
- **[ArUco Library](https://sourceforge.net/projects/aruco)**: Required for marker detection.

### Building

1. Clone this repository into your ROS2 workspace:
    ```sh
    cd <your_workspace>
    git clone https://github.com/dimianx/aruco_fractal_tracker.git
    ```
2. Navigate to the workspace root and build the package:
    ```sh
    cd ~/ros2_ws
    colcon build
    ```

3. Source your workspace:
    ```sh
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

### Running the Node

To run the ArUco Fractal Tracker node, use the following command:
```sh
ros2 run aruco_fractal_tracker aruco_fractal_tracker_node --ros-args -p marker_configuration:=<path_to_marker_config> -p marker_size:=<marker_size> -p cam_params_file:=<path_to_cam_params> --remap image_input_topic:=<your_camera_image_topic> --remap image_output_topic:=<where_to_output_processed_image> --remap poses_output_topic:=<where_to_publish_marker_pose>

```

### Parameters

- `marker_configuration` (string): The configuration file for the ArUco fractal markers.
- `marker_size` (double): The size of the marker in meters.
- `cam_params_file` (string): The camera parameters file (in YAML format).

### Topics

The node uses the following topics:

- `image_input_topic` (input): The topic from which the node subscribes to images for processing.
- `image_output_topic` (output): The topic on which the node publishes images with detected markers and pose information.
- `poses_output_topic` (output): The topic on which the node publishes the pose of the detected marker as a `geometry_msgs::msg::PoseStamped`.

### Example Marker

An example ArUco fractal marker configuration is provided in the `example_marker` folder. This folder contains both the marker image and configuration file necessary for testing the node. To use this example marker:

1. Navigate to the `example_marker` folder.
2. Print out the marker image (`marker.png`) and place it in the camera's field of view.
3. Use the provided configuration file (`marker_configuration.yaml`) to run the node.

# References
- [Romero-Ramire, Francisco J., Rafael Munoz-Salinas, and Rafael Medina-Carnicer. "Fractal markers: A new approach for long-range marker pose estimation under occlusion." IEEE Access 7 (2019): 169908-169919.](https://doi.org/10.1109/ACCESS.2019.2951204)
