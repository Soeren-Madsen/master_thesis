import cv2
import cv2.aruco as aruco

# Create gridboard, which is a set of Aruco markers
gridboard = aruco.GridBoard_create(
        markersX=3, 
        markersY=3, 
        markerLength=0.055, 
        markerSeparation=0.01, 
        dictionary=aruco.Dictionary_get(cv2.aruco.DICT_4X4_50))

# Create an image from the gridboard
img = gridboard.draw(outSize=(988, 1400))
cv2.imwrite("test_gridboard.jpg", img)

# Display the image to us
cv2.imshow('Gridboard', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()