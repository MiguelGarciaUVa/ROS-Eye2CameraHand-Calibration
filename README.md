# ROS-Eye2CameraHand-Calibration
A ROS package for positioning an external camera in the robot's world frame without manual measurements. Using the URDF of the robot's hand camera, it detects Aruco markers to estimate the transformation between the external camera and the robot, publishing the pose as a TF transform for seamless integration.

# Features

✅ Automatic camera positioning using Aruco markers.
✅ Works with Intel RealSense or other RGB cameras.
✅ No manual measurements required—relies on the robot's URDF.
✅ Publishes TF transforms for seamless integration in ROS2.
✅ Saves pose data for further analysis.
