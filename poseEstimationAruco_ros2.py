#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
import math
from rclpy.duration import Duration
import transforms3d.euler as t3d_euler
import transforms3d.quaternions as t3d_quat
import time
import os

class poseEstimation(Node):
    def __init__(self):
        super().__init__('camera_pose_estimation')

        # Initialize variables for camera info and images
        self.camera_info1 = None
        self.camera_info2 = None
        self.image1 = None
        self.image2 = None

        # CvBridge for image conversion
        self.bridge = CvBridge()

        # TF2 Broadcaster (for publishing transforms)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Marker length in meters (adjust as necessary)
        self.marker_length = 0.1  

        # Subscribers for CameraInfo topics
        self.create_subscription(CameraInfo,
                                 '/camera/color/camera_info',
                                 self.camera_info1_callback,
                                 10)
        self.create_subscription(CameraInfo,
                                 '/camera/camera/color/camera_info',
                                 self.camera_info2_callback,
                                 10)

        # Subscribers for Image topics
        self.create_subscription(Image,
                                 '/camera/color/image_raw',
                                 self.image1_callback,
                                 10)
        self.create_subscription(Image,
                                 '/camera/camera/color/image_raw',
                                 self.image2_callback,
                                 10)

        # Timer for processing once data is available (e.g. every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Create a TF2 buffer and listener for transform lookup.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.output_dir = "capturas_aruco"
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.get_logger().info("PoseEstimation node initialized.")

#########################################################################################################################################
# Función que recibe y guarda la información de la cámara (parámetros intrínsecos...) publicada en el topic '/camera/color/camera_info'
#########################################################################################################################################

    def camera_info1_callback(self, msg):
        if self.camera_info1 is None:
            self.get_logger().info("Camera 1 info received.")
        self.camera_info1 = msg
            
    def camera_info2_callback(self, msg):
        if self.camera_info2 is None:
            self.get_logger().info("Camera 2 info received.")
        self.camera_info2 = msg    
            
####################################################            
# Función para capturar imágenes
####################################################
    def image1_callback(self, msg):
        self.image1 = msg

    def image2_callback(self, msg):
        self.image2 = msg

####################################################
# Función para guardar las imagenes y ver que esta pasando
##################################################
    def save_images(self, img1, img2):
        timestamp = int(time.time())
        path1 = os.path.join(self.output_dir, f"image1_{timestamp}.png")
        path2 = os.path.join(self.output_dir, f"image2_{timestamp}.png")
        cv2.imwrite(path1, img1)
        cv2.imwrite(path2, img2)
        self.get_logger().info(f"Images saved: {path1}, {path2}")

####################################################            
# Función para estimar la pose del aruco (estimatePoseSingleMarkers ya no existe en la libreria)
####################################################
    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
         # Define 3D marker points in marker coordinate system (assumed square)
        marker_points = np.array([
            [-marker_size / 2, marker_size / 2, 0],
            [ marker_size / 2, marker_size / 2, 0],
            [ marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0]
        ], dtype=np.float32)

        rvecs = []
        tvecs = []
        # Process each detected marker (here we only use the first marker detected)
        for c in corners:
            ret, rvec, tvec = cv2.solvePnP(marker_points, c, mtx, distortion,
                                            flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ret:
                rvecs.append(rvec)
                tvecs.append(tvec)
            else:
                self.get_logger().warn("solvePnP failed for one marker.")
        return rvecs, tvecs

#############################################################################################################
# Funcion que transforma el tf a matriz
###############################################################################################################
    def transform_to_matrix(self, transform):
            # Convert a TransformStamped.transform to a 4x4 transformation matrix.
        # Extract translation
        trans = np.array([transform.translation.x,
                          transform.translation.y,
                          transform.translation.z])
        # Extract quaternion (assumed in [x, y, z, w] order) and rearrange to [w, x, y, z]
        quat = [transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w]
        quat = [quat[3], quat[0], quat[1], quat[2]]
        # Get rotation matrix from quaternion
        R = t3d_quat.quat2mat(quat)
        # Build 4x4 matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = trans
        return T

#####################################################################################################
#Convierte cuaternio a ángulo de Euler
#####################################################################################################
    def quaternion_to_euler(self, quaternion):
        """
        Convierte un cuaternión a ángulos de Euler en grados.
        
        :param quaternion: Lista o tupla con 4 elementos [x, y, z, w]
        :return: Tupla con 3 elementos (roll, pitch, yaw) en grados
        """
        q = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
        euler_rad = t3d_euler.quat2euler(q, axes='sxyz')
        return tuple(map(math.degrees, euler_rad))
    
####################################################################################################
# Función que calcula la posición de la cámara 2 con respecto a la cámara 1
####################################################################################################
    def timer_callback(self):
        if (self.camera_info1 is None or self.camera_info2 is None or
            self.image1 is None or self.image2 is None):
            self.get_logger().info("Waiting for camera info and images...")
            return

        # Convert ROS Image messages to OpenCV images
        try:
            cv_image1 = self.bridge.imgmsg_to_cv2(self.image1, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(self.image2, "bgr8")
            #self.save_images(cv_image1, cv_image2)
        except Exception as e:
            self.get_logger().error("Error converting image: " + str(e))
            return

        # Get camera matrices and distortion coefficients
        camera_matrix1 = np.array(self.camera_info1.k).reshape(3, 3)
        dist_coeffs1 = np.array(self.camera_info1.d)
        camera_matrix2 = np.array(self.camera_info2.k).reshape(3, 3)
        dist_coeffs2 = np.array(self.camera_info2.d)
        self.get_logger().info("Camera matrices loaded.")

        # Initialize ArUco detector
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        try:
            parameters = aruco.DetectorParameters_create()
        except AttributeError:
            parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Detect markers in both images
        corners1, ids1, _ = detector.detectMarkers(cv_image1)
        corners2, ids2, _ = detector.detectMarkers(cv_image2)

        if not corners1 or not corners2:
            self.get_logger().info("Markers not detected in both images.")
            return

        # Estimate marker poses in both cameras (using the first detected marker)
        rvecs1, tvecs1 = self.estimatePoseSingleMarkers(corners1, self.marker_length,
                                                            camera_matrix1, dist_coeffs1)
        rvecs2, tvecs2 = self.estimatePoseSingleMarkers(corners2, self.marker_length,
                                                            camera_matrix2, dist_coeffs2)

        if not rvecs1 or not rvecs2:
            self.get_logger().warn("Pose estimation failed for one of the cameras.")
            return

        # Lookup transform from base_link to camera 1 frame
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_color_frame',  # adjust this frame if needed
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            T_world_camera1 = self.transform_to_matrix(trans.transform)
        except Exception as e:
            self.get_logger().warn("Could not get transform for camera 1: " + str(e))
            return

        # Compute transformation for marker in camera 1 frame
        R_camera1_marker, _ = cv2.Rodrigues(rvecs1[0])
        T_camera1_marker = np.hstack((R_camera1_marker, tvecs1[0].reshape(3, 1)))
        T_camera1_marker = np.vstack((T_camera1_marker, [0, 0, 0, 1]))
        # Marker pose in world frame
        T_world_marker = T_world_camera1 @ T_camera1_marker

        # Compute transformation for marker in camera 2 frame
        R_camera2_marker, _ = cv2.Rodrigues(rvecs2[0])
        T_camera2_marker = np.hstack((R_camera2_marker, tvecs2[0].reshape(3, 1)))
        T_camera2_marker = np.vstack((T_camera2_marker, [0, 0, 0, 1]))
        # Inverse to get marker-to-camera transform
        T_marker_camera2 = np.linalg.inv(T_camera2_marker)
        # Camera 2 pose in world frame
        T_world_camera2 = T_world_marker @ T_marker_camera2

        # Extract translation (camera 2 position in world)
        trans_world_camera2 = T_world_camera2[:3, 3]
        # Extract rotation as quaternion from the rotation matrix part
        q = t3d_quat.mat2quat(T_world_camera2[:3, :3])
        # transforms3d returns quaternion in [w, x, y, z]; convert to [x, y, z, w]
        q = [q[1], q[2], q[3], q[0]]

        euler_angles = self.quaternion_to_euler(q)
        self.get_logger().info(f"Camera 2 Position: {trans_world_camera2}")
        self.get_logger().info(f"Camera 2 Rotation (Euler): {euler_angles}")
        
        # Guardar la posición y orientación en un archivo .npy
        pose_data = {
            "position": trans_world_camera2,
            "orientation_quaternion": q,
            "orientation_euler": euler_angles
        }

        np.save("camera2_pose.npy", pose_data)
        self.get_logger().info(f"Camera 2 pose saved to camera2_pose.npy")
        
        # Publish the transform from base_link to camera_2_depth_optical_frame
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = 'base_link'
        t_msg.child_frame_id = 'camera_2_depth_optical_frame'
        t_msg.transform.translation.x = float(trans_world_camera2[0])
        t_msg.transform.translation.y = float(trans_world_camera2[1])
        t_msg.transform.translation.z = float(trans_world_camera2[2])
        t_msg.transform.rotation.x = float(q[0])
        t_msg.transform.rotation.y = float(q[1])
        t_msg.transform.rotation.z = float(q[2])
        t_msg.transform.rotation.w = float(q[3])
        self.br.sendTransform(t_msg)

        # Optionally, reset images so that we process only new frames next time
        self.image1 = None
        self.image2 = None
        
def main(args=None):
        rclpy.init(args=args)
        node = poseEstimation()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down node...")
        node.destroy_node()
        rclpy.shutdown()
            
if __name__ == '__main__':
    main()
