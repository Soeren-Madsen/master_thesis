#!/usr/bin/env python
  
  
from __future__ import print_function 
import cv2 
import numpy as np
from scipy.spatial.transform import Rotation as R
import math 
import time


 
class Aruco_pose():
    def __init__(self, gazebo):
        self.gazebo = gazebo
        # Side length of the ArUco marker in meters 
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create() 
        # Calibration parameters yaml file
        if gazebo:
            #Works for distance
            # camera_calibration_parameters_filename = 'calibration_chessboard.yaml'
            # aruco_marker_side_length =0.097#0.097#0.66
            # aruco_marker_space = 0.025#0.025#0.066

            #Works for x,y correction
            camera_calibration_parameters_filename = 'calibration_chessboard_gazebo.yaml'
            aruco_marker_side_length = 0.455
            aruco_marker_space = 0.09

            self.board = cv2.aruco.GridBoard_create(2, 2, aruco_marker_side_length, aruco_marker_space, self.arucoDict) #Gazebo
        else:
            camera_calibration_parameters_filename = 'calibration_chessboard_real.yaml'
            aruco_marker_side_length =0.055
            aruco_marker_space = 0.014
            self.board = cv2.aruco.GridBoard_create(3, 3, aruco_marker_side_length, aruco_marker_space, self.arucoDict) #Real life

        cv_file = cv2.FileStorage(camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()
            
        # Load the ArUco dictionary


        #Create new custom aruco board. Location of aruco marker corners. Only if in 3D
        #board_corners = [np.array([[0.007,0.096,0.0],[0.099,0.096,0.0],[0.099,0.005,0.0],[0.007,0.005,0.0]],dtype=np.float32),
        #np.array([[0.0,0.099,-0.006],[0.0,0.005,-0.006],[0.0,0.005,-0.096],[0.0,0.099,-0.096]],dtype=np.float32),
        #np.array([[0.007,0.0,-0.006],[0.099,0.0,-0.006],[0.099,0.0,-0.097],[0.007,0.0,-0.097]],dtype=np.float32)]

          #Id's for aruco marker on board
        #board_ids = np.array([[0],[1],[2]], dtype=np.int32)

        #Creating board
        #self.board = cv2.aruco.Board_create(board_corners,self.arucoDict, board_ids )

        #Creating a standard board, no custom
        
        

        self.rvecs = None
        self.tvecs = None    

        self.transform_translation_x = 0
        self.transform_translation_y = 0
        self.transform_translation_z = 10
        self.roll_x = 0
        self.pitch_y = 0
        self.yaw_z = 0



    def euler_from_quaternion(self, x, y, z, w):
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

    def calc_euler(self, frame):
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(frame, self.arucoDict, parameters=self.arucoParams)

        if marker_ids is not None and marker_ids is not 17:
    
            #print(marker_ids)
            # Draw a square around detected markers in the video frame
            cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
            
            # Get the rotation and translation vectors
            #print(dst)
            success, rvecs, tvecs = cv2.aruco.estimatePoseBoard(corners, marker_ids, self.board, self.mtx, self.dst, rvec=None, tvec=None)

            if success > 0:
                frame = cv2.aruco.drawAxis(frame, self.mtx, self.dst, rvecs, tvecs, 0.05)
            
                for i, marker_id in enumerate(marker_ids):
            
                    # Store the translation (i.e. position) information
                    self.transform_translation_x = tvecs[0]
                    self.transform_translation_y = tvecs[1]
                    self.transform_translation_z = tvecs[2] 
                    if self.gazebo:
                        self.transform_translation_z = tvecs[2] + 1.257#The difference between the true measurement and aruco placement on the ship in gazebo
            
                    # Store the rotation information
                    rotation_matrix = np.eye(4)
                    try:
                        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs)[0]
                        r = R.from_matrix(rotation_matrix[0:3, 0:3])
                        quat = r.as_quat()   
                        # Quaternion format     
                        transform_rotation_x = quat[0] 
                        transform_rotation_y = quat[1] 
                        transform_rotation_z = quat[2] 
                        transform_rotation_w = quat[3]
                        # Euler angle format in radians
                        self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(transform_rotation_x, 
                                                                        transform_rotation_y, 
                                                                        transform_rotation_z, 
                                                                        transform_rotation_w)
                        #self.roll_x = math.degrees(self.roll_x)
                        #self.pitch_y = math.degrees(self.pitch_y)
                        self.yaw_z = math.degrees(self.yaw_z)
                        #print("transform_translation_x: {}".format(self.transform_translation_x))
                        #print("transform_translation_y: {}".format(self.transform_translation_y))
                        #print("transform_translation_z: {}".format(self.transform_translation_z))
                        #print("roll_x: {}".format(self.roll_x))
                        #print("pitch_y: {}".format(self.pitch_y))
                        #print("yaw_z: {}".format(self.yaw_z))
                        #print("quaternion 1: {}".format(quat[0]))
                        #print("quaternion 2: {}".format(quat[1]))
                        #print("quaternion 3: {}".format(quat[2]))
                        #print("quaternion 4: {}".format(quat[3]))
                        
                        
                        # Draw the axes on the marker
                        #cv2.aruco.drawAxis(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
                    except:
                        print("Something went wrong with Rodrigues")

            else:
                print("No Aruco markers found!")
                success = False
        else:
            success = False

        #cv2.imshow('frame',frame)
        return success, self.transform_translation_z, self.yaw_z, self.transform_translation_y, self.transform_translation_x, self.roll_x, self.pitch_y #Using last value if no aruco marker is detected

 
def main():
    aru = Aruco_pose(False)
    # Start the video stream
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    while(True):
    
        ret, frame = cap.read()
        aru.calc_euler(frame)
        #cv2.imshow('frame', frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break

    
    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
  print(__doc__)
  main()
