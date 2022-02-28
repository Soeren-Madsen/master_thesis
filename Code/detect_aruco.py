import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
import math # Math library

if __name__ == '__main__':
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    cap = cv2.VideoCapture(0)
    while(True):
        
        ret, frame = cap.read()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams) 
        cv2.imshow("image", frame)
        print(ids) 


    image = cv2.imread("aruco_13.png")
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    cv2.imshow("image", image)
    print(ids)
    cv2.waitKey(0)

    print("test")