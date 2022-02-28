from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
import time





class aruco_detect:
    def __init__(self, calibration_file):
        self.aruco_marker_side_length = 0.096
        self.camera_calibration_parameters_filename = calibration_file

        cv_file = cv2.FileStorage(
        self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()

        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        self.cap = cv2.VideoCapture(0) #Setup webcam
        print("Initializing complete")

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians

    def capture_frame(self):
        ret, self.frame = self.cap.read()

    def rot_vec_to_euler(self, rotation_vector):
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rotation_vector))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        self.quat = r.as_quat()   
         
        # Quaternion format     
        transform_rotation_x = self.quat[0] 
        transform_rotation_y = self.quat[1] 
        transform_rotation_z = self.quat[2] 
        transform_rotation_w = self.quat[3] 
         
        # Euler angle format in radians
        #print(self.quat)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w)
         
        self.roll_x = math.degrees(roll_x)
        self.pitch_y = math.degrees(pitch_y)
        self.yaw_z = math.degrees(yaw_z)
        
    def estimate_pose(self):
        (self.corners, self.marker_ids, rejected) = cv2.aruco.detectMarkers(
        self.frame, self.arucoDict, parameters=self.arucoParams)
        if self.marker_ids is not None and self.marker_ids is not 17:
            cv2.aruco.drawDetectedMarkers(self.frame, self.corners, self.marker_ids)
            corners_pnp = np.asarray(self.corners)[0][0]
            print(corners_pnp.shape)
            imagePoints = np.ascontiguousarray(corners_pnp[:,:2]).reshape((4,1,2))
            self.points_3d = np.array([(-self.aruco_marker_side_length/2, self.aruco_marker_side_length/2, 0.0),(-self.aruco_marker_side_length/2, -self.aruco_marker_side_length/2, 0.0), (self.aruco_marker_side_length/2, -self.aruco_marker_side_length/2, 0.0),(self.aruco_marker_side_length/2,self.aruco_marker_side_length/2,0.0)])
            success, rotation_vector, translation_vector = cv2.solvePnP(self.points_3d, imagePoints, self.mtx, self.dst, flags=0)
            if success:
                #print(rotation_vector)
                print(translation_vector)
                #print(self.points_3d)
                #print(corners_pnp)
                cv2.aruco.drawAxis(self.frame, self.mtx, self.dst, rotation_vector, translation_vector, 0.05)
                self.rot_vec_to_euler(rotation_vector)
                print("roll_x: {}".format(self.roll_x))
                print("pitch_y: {}".format(self.pitch_y))
                print("yaw_z: {}".format(self.yaw_z))

    

if __name__ == '__main__':
    aru=aruco_detect('calibration_chessboard.yaml')
    while(True):
        aru.capture_frame()
        aru.estimate_pose()
        cv2.imshow('frame',aru.frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
  
    # Close down the video stream
    aru.cap.release()
    cv2.destroyAllWindows()