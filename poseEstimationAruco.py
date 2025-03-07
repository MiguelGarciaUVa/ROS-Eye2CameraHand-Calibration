#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import tf
import tf2_ros
import transforms3d.quaternions as t3d_quat
from cv_bridge import CvBridge
from geometry_msgs import msg
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import *

class poseEstimation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_pose_estimation', anonymous=True)
        self.camera_info1 = None
        self.camera_info2 = None
        self.image1 = None
        self.image2 = None

        # Subscribe to camera info topic
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info1_callback)
        rospy.Subscriber('/camera_2/color/camera_info', CameraInfo, self.camera_info2_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image1_callback)
        rospy.Subscriber('/camera_2/color/image_raw', Image, self.image2_callback)

        # Inicializar la clase CvBridge
        self.bridge = CvBridge()
        
        # Setup tf listener and broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Marker length in meters
        self.marker_length = 0.1  

        self.output_dir = "capturas_aruco"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        rospy.Timer(rospy.Duration(1.0), self.calculate_transform)
        rospy.loginfo("PoseEstimation node initialized.")

#########################################################################################################################################
# Función que recibe y guarda la información de la cámara (parámetros intrínsecos...) publicada en el topic '/camera/color/camera_info'
#########################################################################################################################################

    def camera_info1_callback(self, camera_info1):
        if self.camera_info1 is None:
            self.camera_info1 = camera_info1
            rospy.loginfo("Información de la cámara recibida. Listo para procesar imágenes.")
            
    def camera_info2_callback(self, camera_info2):
        if self.camera_info2 is None:
            self.camera_info2 = camera_info2
            rospy.loginfo("Información de la cámara recibida. Listo para procesar imágenes.")       
            
####################################################            
# Función para capturar imágenes
####################################################
    def image1_callback(self, msg):
        self.image1 = msg

    def image2_callback(self, msg):
        self.image2 = msg

    def save_images(self, img1, img2):
            timestamp = int(time.time())
            path1 = os.path.join(self.output_dir, f"image1_{timestamp}.png")
            path2 = os.path.join(self.output_dir, f"image2_{timestamp}.png")
            cv2.imwrite(path1, img1)
            cv2.imwrite(path2, img2)
            rospy.loginfo(f"Images saved: {path1}, {path2}")

    

####################################################            
# Función para estimar la pose del aruco (estimatePoseSingleMarkers ya no existe en la libreria)
####################################################
    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
           corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        rvecs = []
        tvecs = []
        for c in corners:
            _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
        return rvecs, tvecs

#############################################################################################################
# Funcion que transforma el tf a matriz
###############################################################################################################
    def transform_to_matrix(self, transform):
            trans = [transform.translation.x, transform.translation.y, transform.translation.z]
            rot = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
            T = tf.transformations.quaternion_matrix(rot)
            T[0:3, 3] = trans
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
        # Desempaqueta el cuaternión
        x, y, z, w = quaternion
        
        # Convierte el cuaternión a matriz de rotación
        rotation_matrix = tf.transformations.quaternion_matrix([x, y, z, w])
        
        # Convierte la matriz de rotación a ángulos de Euler en radianes
        euler_angles_rad = tf.transformations.euler_from_matrix(rotation_matrix)
        
        # Convierte los ángulos de Euler de radianes a grados
        euler_angles_deg = tuple(map(math.degrees, euler_angles_rad))
        
        return euler_angles_deg
    
####################################################################################################
# Función que calcula la posición de la cámara 2 con respecto a la cámara 1
####################################################################################################
    def CalculateTransform(self, event):
        if self.camera_info1 is None or self.camera_info2 is None or self.image1 is None or self.image2 is None:
            rospy.loginfo("Waiting for camera info and images...")
            return

        try:
            cv_image1 = self.bridge.imgmsg_to_cv2(self.image1, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(self.image2, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: " + str(e))
            return
        
        camera_matrix1 = np.array(self.camera_info1.K).reshape(3, 3)
        dist_coeffs1 = np.array(self.camera_info1.D)
        camera_matrix2 = np.array(self.camera_info2.K).reshape(3, 3)
        dist_coeffs2 = np.array(self.camera_info2.D)
        
        # Detect markers
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        corners1, ids1, _ = detector.detectMarkers(cv_image1)
        corners2, ids2, _ = detector.detectMarkers(cv_image2)

        if not corners1 or not corners2:
            rospy.loginfo("Markers not detected in both images.")
            return
        
        rvecs1, tvecs1 = self.estimatePoseSingleMarkers(corners1, self.marker_length, camera_matrix1, dist_coeffs1)
        rvecs2, tvecs2 = self.estimatePoseSingleMarkers(corners2, self.marker_length, camera_matrix2, dist_coeffs2)

        if not rvecs1 or not rvecs2:
            rospy.logwarn("Pose estimation failed for one of the cameras.")
            return
        
        # Known pose of the first camera in the world frame
        try:
            trans, rot = self.tf_listener.lookupTransform('base_link', 'camera_color_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not get transform for camera 1.")
            return
        
        T_world_camera1 = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

        # Compute transformation from camera1 to marker
        R_camera1_marker, _ = cv2.Rodrigues(rvecs1[0])
        T_camera1_marker = np.hstack((R_camera1_marker, tvecs1[0].reshape(3, 1)))
        T_camera1_marker = np.vstack((T_camera1_marker, [0, 0, 0, 1]))

        # Compute transformation from world to marker
        T_world_marker = np.dot(T_world_camera1, T_camera1_marker)

        # Compute transformation from camera2 to marker
        R_camera2_marker, _ = cv2.Rodrigues(rvecs2[0])
        T_camera2_marker = np.hstack((R_camera2_marker, tvecs2[0].reshape(3, 1)))
        T_camera2_marker = np.vstack((T_camera2_marker, [0, 0, 0, 1]))

        # Compute transformation from world to camera2
        T_marker_camera2 = np.linalg.inv(T_camera2_marker)
        T_world_camera2 = np.dot(T_world_marker, T_marker_camera2)

        # Extract translation and rotation
        trans_world_camera2 = T_world_camera2[:3, 3]
        q = t3d_quat.mat2quat(T_world_camera2[:3, :3])
        # transforms3d returns quaternion in [w, x, y, z]; convert to [x, y, z, w]
        q_converted = [q[1], q[2], q[3], q[0]]

        euler_angles = self.quaternion_to_euler(q_converted)

        # Use the transform directly
        print("Translation: ", trans_world_camera2)
        print("Rotation: ", euler_angles)
        
        # Guardar la posición y orientación en un archivo .npy
        pose_data = {
            "position": trans_world_camera2,
            "orientation_quaternion": q_converted,
            "orientation_euler": euler_angles
        }

        np.save("camera2_pose.npy", pose_data)
        rospy.loginfo(f"Camera 2 pose saved to camera2_pose.npy")
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(trans_world_camera2, q_converted, rospy.Time.now(), 'camera_2_color_frame', 'base_link')
        
if __name__ == '__main__':
    try:
        PoseEstimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
