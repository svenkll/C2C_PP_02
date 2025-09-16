from basecar import BaseCar
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera
import time
import json
import numpy as np
import matplotlib.pylab as plt
import cv2

class CameraCar(BaseCar):

    def __init__(self, front, back, camera, values_to_log):
        super().__init__(front, back, values_to_log)
        self.camera = camera
        self.picture_collect()
        print("CameraCar erzeugt")

    def picture_handler(self):
        img = self.camera.get_frame()
        self.camera.release()
        cv2.imwrite("Bild.jpg", img)
        img_small = cv2.resize(img, None, fx=0.25, fy=0.25)
        #plt.imshow(img)




#main()

front = FrontWheels()
back = BackWheels()
cam = Camera(devicenumber = 0,
            buffersize = 10,
            skip_frame = 0,
            height = 480,
            width = 640,
            flip = True,
            colorspace = 'rgb')


cam_car = CameraCar(front,back,cam, [])