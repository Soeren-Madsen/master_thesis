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



if __name__ == '__main__':
    cam = Cam()
    i=0
    while True:
        frame = cam.get_img()
        cv2.imwrite("image" + i, frame)
        cv2.waitKey(0)