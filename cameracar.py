from basecar import BaseCar
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera
import time
import json
import numpy as np
import matplotlib.pylab as plt
import cv2
import time

class CameraCar(BaseCar):

    def __init__(self, front, back, camera, values_to_log):
        super().__init__(front, back, values_to_log)
        self.camera = camera
        self.index = 0
        print("CameraCar erzeugt")

    def picture_handler(self, lower_blue_input=[90, 60, 60], upper_blue_input=[130, 255, 255] ):
        self.index += 1
        img = self.camera.get_frame()
        self.camera.release()
        #muss noch angepasst werden, wegen Dateien Benennung Ziel: Bild0x_Winkel_Grad
        cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild.jpg", img)
        # cv2.imwrite("Bild.jpg", img)
        img_read = cv2.imread("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild.jpg")
        img_small = cv2.resize(img_read, None, fx=0.25, fy=0.25) # 50% prozent vom originalen Bild, fx, fy als parameter
        cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_small.jpg", img_small)
        print(img_small.shape)
        img_cropped = img_small.copy()[40:90, :, :] #40:90 und 20:150 als variablen oder einstellbar
        cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_cropped.jpg", img_cropped)
        hsv = cv2.cvtColor(img_cropped,cv2.COLOR_BGR2HSV)
        print(hsv.shape)
        lower_blue = np.array(lower_blue_input)
        print(lower_blue)
        upper_blue = np.array(upper_blue_input)
        print(upper_blue)
        img_filtered = cv2.inRange(hsv, lower_blue, upper_blue)
        img_edges = cv2.Canny(img_filtered, 900, 1000)
        lines = cv2.HoughLinesP(img_edges,  1, np.pi / 180, threshold=30, minLineLength=25, maxLineGap=10) # extern Input ?
        #zu entscheiden, welche angpasst Bild soll gespecihert werden
        cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures_modified/Bild_angepasst.jpg", img_edges)
        print(lines)
        return lines, img
        #plt.imshow(img)

 # lenkwinkel berechnen und zurückgeben       
    def angle_calc(self, lines):
        rechts = []
        links = []
        if lines is not None:
            for line in lines:
                xEnd, yEnd, xStart, yStart = line[0]
                
                if xEnd < 60:
                    print("Links")
                    links.append(line[0])
                elif xEnd > 60:
                    print("Rechts")
                    rechts.append(line[0])
        if len(rechts) == 1:
            rdurch = rechts
            print(rdurch)
            alpha2 = np.arctan((rdurch[0][3]-rdurch[0][1])/(rdurch[0][2]-rdurch[0][0]))
            alpha2 = np.degrees(alpha2)
            print(f"Alpha2 {alpha2}")
        elif len(rechts) >= 2:
            rdurch = np.mean(rechts, axis=0, keepdims=False) #muss noch parameter angepasst werden
            #(rechts[0] + rechts[1])/2
            print(rdurch)
            alpha2 = np.arctan((rdurch[3]-rdurch[1])/(rdurch[2]-rdurch[0]))
            alpha2 = np.degrees(alpha2)
            print(f"Alpha2 {alpha2}")
        else:
            print("no right lane")
            alpha2 = 60 # muss noch überprüft werden

        #mit einer linie probieren
        if len(links) == 1:
            ldurch = links 
            print(f"Ldurch {ldurch}")
            alpha = np.arctan((ldurch[0][3]-ldurch[0][1])/(ldurch[0][2]-ldurch[0][0]))
            alpha = abs(np.degrees(alpha))
            print(f"Alpha {alpha}")
        elif len(links) >= 2:
            ldurch = np.mean(links, axis=0, keepdims=False)
            print(f"Ldurch {ldurch}")
            alpha = np.arctan((ldurch[3]-ldurch[1])/(ldurch[2]-ldurch[0]))
            alpha = abs(np.degrees(alpha))
            print(f"Alpha {alpha}")
        else:
            print("no left lane")
            alpha = 60 # muss noch überprüft werden

        # Berechnung des Lenkwinkels

        diffangle = 90 + (alpha2-alpha) # vielleicht muss mit Betrag (Vorzeichen) bearbeitet werden
        print(f"Diffwinkel   {diffangle}")

        return diffangle

    def save_picture(self, img, diffangle):
        #picture_index = self.index #??
        int_angle = int(diffangle)
        picture_path = f"/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_{self.index}_{int_angle}.jpg"
        cv2.imwrite(picture_path, img)
        self.index += 1
        
        
        
        
    def video_handler(self, lower_blue_input=[90, 60, 60], upper_blue_input=[130, 255, 255]):
        # while True:
            video_stream = self.camera.get_frame()
            img = video_stream
            video_small = cv2.resize(video_stream, None, fx=0.25, fy=0.25) # 50% prozent vom originalen Bild, fx, fy als parameter
            video_cropped = video_small.copy()[40:90, :, :] #40:90 und 20:150 als variablen oder einstellbar
            video_hsv = cv2.cvtColor(video_cropped,cv2.COLOR_BGR2HSV)
            lower_blue = np.array(lower_blue_input)
            upper_blue = np.array(upper_blue_input)
            video_filtered = cv2.inRange(video_hsv, lower_blue, upper_blue)
            video_edges = cv2.Canny(video_filtered, 900, 1000)
            lines = cv2.HoughLinesP(video_edges,  1, np.pi / 180, threshold=30, minLineLength=25, maxLineGap=10) # extern Input ?
            return lines, img

   
if __name__ == "__main__":

    front = FrontWheels()
    back = BackWheels()
    cam = Camera(devicenumber = 0,
                buffersize = 10,
                skip_frame = 0,
                height = 480,
                width = 640,
                flip = True,
                )



    cam_car = CameraCar(front,back,cam, [])
    cam_car.drive(30,90)
    i = 0
    while i < 15:
        lines, img = cam_car.video_handler()
        diffangle = cam_car.angle_calc(lines)
        cam_car.drive(new_angle=diffangle)
        cam_car.save_picture(img, diffangle)
        time.sleep(0.2)
        i +=1 
    cam_car.stop()


    # lines, img = cam_car.picture_handler()
    # cam_car.angle_calc(lines)