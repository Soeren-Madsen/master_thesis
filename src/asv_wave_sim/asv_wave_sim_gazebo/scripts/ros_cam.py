import cv2

class Cam():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)

    def get_img(self):
        ret, frame = cap.read()
        if ret:
            return frame
        else:
            print("no image")   
        #cv2.imwrite('image.jpg', frame)