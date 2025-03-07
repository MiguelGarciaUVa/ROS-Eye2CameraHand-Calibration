# ROS-Eye2CameraHand-Calibration
A ROS package for positioning an external camera in the robot's world frame without manual measurements. Using the URDF of the robot's hand camera, it detects Aruco markers to estimate the transformation between the external camera and the robot, publishing the pose as a TF transform for seamless integration.
# ROS Package for External Camera Positioning

This ROS package is designed to position an external camera within the robot's world frame without manual measurements. The package uses the URDF of the robot's hand camera to detect Aruco markers and estimate the transformation between the external camera and the robot. The pose of the external camera is published as a TF transform, enabling seamless integration with other ROS components.

## Features

- **Camera Positioning**: Automatically estimates the position of an external camera with respect to the robot's world frame.
- **Aruco Marker Detection**: Utilizes Aruco markers to calculate transformations.
- **TF Broadcasting**: Publishes the estimated transformation as a TF transform, enabling integration with other parts of the robot system.
- **Support for Dual Cameras**: Can be extended for systems with multiple cameras for better accuracy and depth.

## Installation

Make sure you have ROS 2 (e.g., Humble) installed and set up properly. Follow the steps below to install the package:

1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/your-repository.git
   
# Features

✅ Automatic camera positioning using Aruco markers.
✅ Works with Intel RealSense or other RGB cameras.
✅ No manual measurements required—relies on the robot's URDF.
✅ Publishes TF transforms for seamless integration in ROS2.
✅ Saves pose data for further analysis.
