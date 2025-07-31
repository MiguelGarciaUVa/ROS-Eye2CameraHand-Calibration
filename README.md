# ROS-Eye2CameraHand-Calibration
This ROS package is designed to position an external camera within the robot's world frame without manual measurements. The package uses the URDF of the robot's hand camera to detect Aruco markers and estimate the transformation between the external camera and the robot. The pose of the external camera is published as a TF transform, enabling seamless integration with other ROS components.

## Features

- ✅ **Camera Positioning**: Automatic camera positioning using Aruco markers.
- ✅ **Camera Compatibility**: Works with Intel RealSense or other RGB cameras.
- ✅ **Aruco Marker Detection**: Eliminates the need for manual measurements by utilizing the robot’s URDF.
- ✅ **TF Broadcasting**: Publishes the estimated transformation as a TF transform, enabling integration with other parts of the robot system.
- ✅ **Pose Data Saving**: Saves pose data for further analysis.

## Installation
### ROS 1
Make sure you have ROS 1 (e.g., Noetic) installed and set up properly. Follow the steps below to install the package:
1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/MiguelGarciaUVa/ROS-Eye2CameraHand-Calibration.git
   ```
2. Install Dependencies:

3. Create a ROS Workspace:

4. Source the Workspace:

5. Run the Node:

### ROS 2
Make sure you have ROS 2 (e.g., Humble) installed and set up properly. Follow the steps below to install the package:
1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/MiguelGarciaUVa/ROS-Eye2CameraHand-Calibration.git
   ```
2. Install Dependencies:

3. Create a ROS Workspace:

4. Source the Workspace:

5. Run the Node: 

## Usage
Setup
1. Launch External Camera: Make sure the external camera is correctly connected and configured within your ROS environment. If using Intel RealSense, launch the appropriate ROS node for the camera.

2. Launch Robot's TF Node: Ensure that the robot's transformation data is being published. This should include the robot’s URDF and its relevant TF frames.

3. Launch Robot Camera: Start the robot's camera (hand camera) and ensure it is streaming data for Aruco marker detection.

4. Place Aruco Markers: Position the Aruco marker on the environment where both cameras can see it.

5. Launch the Calibration Program: Finally, launch the calibration node to detect Aruco markers and compute the transformation between the external camera and the robot’s hand camera:
 
## Troubleshooting
- **Camera not detected**: Ensure that the camera drivers are properly installed and the camera is connected correctly.
- **Markers not detected**: Make sure the Aruco markers are clearly visible and correctly positioned. Adjust camera settings (e.g., focus, exposure) if necessary.
- **TF transform issues**: Verify that the robot’s TF tree is correctly set up and broadcasting the necessary frames.

