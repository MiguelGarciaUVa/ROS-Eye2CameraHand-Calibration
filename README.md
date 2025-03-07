# ROS-Eye2CameraHand-Calibration
This ROS package is designed to position an external camera within the robot's world frame without manual measurements. The package uses the URDF of the robot's hand camera to detect Aruco markers and estimate the transformation between the external camera and the robot. The pose of the external camera is published as a TF transform, enabling seamless integration with other ROS components.

## Features

-✅ **Camera Positioning**: Automatic camera positioning using Aruco markers.
-✅ **Camera Compatibility**: Works with Intel RealSense or other RGB cameras.
-✅ **Aruco Marker Detection**: Eliminates the need for manual measurements by utilizing the robot’s URDF.
-✅ **TF Broadcasting**: Publishes the estimated transformation as a TF transform, enabling integration with other parts of the robot system.
-✅ **Pose Data Saving**: Saves pose data for further analysis.

## Installation
### ROS 1
Make sure you have ROS 1 (e.g., Noetic) installed and set up properly. Follow the steps below to install the package:
1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/your-repository.git
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
   git clone https://github.com/your-username/your-repository.git
   ```
2. Install Dependencies:

3. Create a ROS Workspace:

4. Source the Workspace:

5. Run the Node: 

## Usage
1. Lanzar cámara externa en el entorno de ROS

2. Lanzar TF del robot con la cámara

3. Lanzar cámara del robot

4. Lanzar programa
 


