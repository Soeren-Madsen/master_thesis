import cv2

class Cam():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def get_img(self):
        ret, frame = self.cap.read()
        if ret:
            return frame
        else:
            print("no image")   



if __name__ == '__main__':
    cam = Cam()
    i=0
    while True:
        input("Press enter to continue...")
        frame = cam.get_img()
        name = "image" + str(i) + ".jpg"
        print(name)
        cv2.imwrite(name, frame)
        print("Saving img")
        i = i+1
