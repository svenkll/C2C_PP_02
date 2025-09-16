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
        # cv2.imwrite("Bild.jpg", img)
        img_small = cv2.resize(img, None, fx=0.25, fy=0.25)
        img_cropped = img_small[25:90, 10:140, :]
        hsv = cv2.cvtColor(img_cropped,cv2.COLOR_BGR2HSV)
        lower_blue = np.array([30, 50, 50]) # extern Input
        upper_blue = np.array([60, 200, 200]) # extern Input
        img_filtered = cv2.inRange(img_cropped, lower_blue, upper_blue)
        img_edges = cv2.Canny(img_filtered, 900, 1000)
        lines = cv2.HoughLinesP(img_edges,  1, np.pi / 180, threshold=30, minLineLength=25, maxLineGap=10) # extern Input ?
        return lines
        #plt.imshow(img)
        
    def angle_calc(self, HoughLinesP)
        pass
#         if lines is not None:
#           rechts = []
#           links = []
#           for line in lines:
#               xEnd, yEnd, xStart, yStart = line[0]
#               länge = np.sqrt((rxEnd-rxStart)**2 + (ryEnd - ryStart)**2)
#               if xStart < 60:
#                   print("Links")
#                   links.append(line[0])
#               elif xStart > 60:
#                   print("Rechts")
#                   rechts.append(line[0])
#       rdurch = (rechts[0] + rechts[1])/2
#       ldurch = (links[0] + links[1])/2
#       print(rdurch)
#       print(ldurch)

#       alpha2 = np.arctan((rdurch[3]-rdurch[1])/(rdurch[2]-rdurch[0]))
#       alpha2 = np.degrees(alpha2)
#       alpha2

#       alpha = np.arctan((ldurch[3]-ldurch[1])/(ldurch[2]-ldurch[0]))
#       alpha = abs(np.degrees(alpha))
#       alpha

#       diffangle = alpha2-alpha

# lenkwinkel berechnen und zurückgeben

    



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