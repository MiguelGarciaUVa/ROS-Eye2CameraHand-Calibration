#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from tf.transformations import *
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
import tf.transformations
import math
import geometry_msgs

class poseEstimation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_pose_estimation', anonymous=True)
        self.camera_info1 = None
        self.camera_info2 = None
        
        # Subscribe to camera info topic
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info1_callback)
        rospy.Subscriber('/camera_2/color/camera_info', CameraInfo, self.camera_info2_callback)
        
        # Inicializar la clase CvBridge
        self.bridge = CvBridge()
        
        # Setup tf listener and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        # Marker length in meters
        self.marker_length = 0.1  

        # Wait until the camera info is received
        while not rospy.is_shutdown() and (self.camera_info1 is None and self.camera_info2 is None):
            rospy.sleep(0.1)
        

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
    def capture_images(self):
    
        # Captura imagen de la cámara 1
        image_cam1 = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=None)
        rospy.loginfo("Imagen de la cámara 1 capturada.")
        image_cam1 = self.bridge.imgmsg_to_cv2(image_cam1, "bgr8")
    
        # Captura imagen de la cámara 2
        image_cam2 = rospy.wait_for_message('/camera_2/color/image_raw', Image, timeout=None)
        rospy.loginfo("Imagen de la cámara 2 capturada.")
        image_cam2 = self.bridge.imgmsg_to_cv2(image_cam2, "bgr8")
        
        return image_cam1, image_cam2

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
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

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
    def CalculateTransform(self):
        # Load camera calibration parameters
        # self.camera_matrix1 = self.camera_info1.K
        # self.dist_coeffs1 = self.camera_info1.D
        # self.camera_matrix2 = self.camera_info2.K
        # self.dist_coeffs2 = self.camera_info2.D
        
        self.camera_matrix1 = np.array(self.camera_info1.K).reshape(3, 3)
        self.dist_coeffs1 = np.array(self.camera_info1.D)
        self.camera_matrix2 = np.array(self.camera_info2.K).reshape(3, 3)
        self.dist_coeffs2 = np.array(self.camera_info2.D)
        
        print(self.camera_matrix1)
        print(self.camera_matrix2)
        # Load images
        image1, image2 = self.capture_images()
        
        
        # Detect markers
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        
        corners1, ids1, _ = detector.detectMarkers(image1)
        corners2, ids2, _ = detector.detectMarkers(image2)
        
        if len(corners1) > 0 and len(corners2) > 0:
            # Estimate pose of the ArUco marker for both cameras
            rvec1, tvec1, _ = self.estimatePoseSingleMarkers(corners1, self.marker_length, self.camera_matrix1, self.dist_coeffs1)
            rvec2, tvec2, _ = self.estimatePoseSingleMarkers(corners2, self.marker_length, self.camera_matrix2, self.dist_coeffs2)
            
            # Known pose of the first camera in the world frame
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'camera_color_frame', rospy.Time())
                print(trans)
                T_world_camera1 = self.transform_to_matrix(trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("No se pudo obtener la transformación de camera_color_frame a base_link.")
                return None
            
            # Compute transformation from camera1 to marker
            R_camera1_marker, _ = cv2.Rodrigues(rvec1[0])
            T_camera1_marker = np.hstack((R_camera1_marker, tvec1[0].reshape(3, 1)))
            T_camera1_marker = np.vstack((T_camera1_marker, [0, 0, 0, 1]))
    
            # Compute transformation from world to marker
            T_world_marker = np.dot(T_world_camera1, T_camera1_marker)
    
            # Compute transformation from camera2 to marker
            R_camera2_marker, _ = cv2.Rodrigues(rvec2[0])
            T_camera2_marker = np.hstack((R_camera2_marker, tvec2[0].reshape(3, 1)))
            T_camera2_marker = np.vstack((T_camera2_marker, [0, 0, 0, 1]))
    
            # Compute transformation from world to camera2
            T_marker_camera2 = np.linalg.inv(T_camera2_marker)
            T_world_camera2 = np.dot(T_world_marker, T_marker_camera2)
    
            # Extract translation and rotation
            trans_world_camera2 = tf.transformations.translation_from_matrix(T_world_camera2)
            rot_world_camera2 = tf.transformations.quaternion_from_matrix(T_world_camera2)
            
            # Use the transform directly
            print("Translation: ", trans_world_camera2)
            print("Rotation: ", self.quaternion_to_euler(rot_world_camera2))
            
            
            # Broadcast the transform
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'camera_2_depth_optical_frame'
            #t.child_frame_id = 'camera_2_link'
            t.transform.translation.x = trans_world_camera2[0]
            t.transform.translation.y = trans_world_camera2[1]
            t.transform.translation.z = trans_world_camera2[2]
            t.transform.rotation.x = rot_world_camera2[0]
            t.transform.rotation.y = rot_world_camera2[1]
            t.transform.rotation.z = rot_world_camera2[2]
            t.transform.rotation.w = rot_world_camera2[3]
            
            while not rospy.is_shutdown():
                self.br.sendTransform(t)
                rospy.sleep(0.1)
            
if __name__ == '__main__':
    poseEstimation = poseEstimation()
    poseEstimation.CalculateTransform()