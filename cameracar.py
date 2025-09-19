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
        #??? liefert Video oder bilder zurück
        self.frame = self.camera.get_frame()
        self.index = 0
        self.upper_blue_input = np.array([90, 60, 60])
        self.lower_blue_input = np.array([130, 255, 255])
        # upper_blue_input in die init für die slider bei Dash // funktionen anpassen auf self. upper
        # lower_blue_input in die init für die slider bei Dash
        print("CameraCar erzeugt")
        
 # werden die Parameter aus Json raus gelesen       
    def farb_config(self):
        try:
            with open("config.json", "r") as ff:
                data = json.load(ff)
        except (FileNotFoundError, json.JSONDecodeError):
            # Datei fehlt oder kaputt → Standardwerte
            print("Keine geeignete Datei config.json gefunden! Default-Werte werden gesetzt.")
            self.upper_blue_input = np.array([90, 60, 60])
            self.lower_blue_input = np.array([130, 255, 255])
        else:
            # Werte aus der Datei übernehmen
            upper_blue_input = data.get("upper_blue_input", [90, 60, 60])
            lower_blue_input = data.get("lower_blue_input", [130, 255, 255])
            
            # einzelne Positionen übernehmen
            self.upper_blue_input = np.array(upper_blue_input)
            self.lower_blue_input = np.array(lower_blue_input)
            
            print("Daten in config.json:")
            print(f"Upper: {self.upper_blue_input}, Lower: {self.lower_blue_input}")

# werden die Bilder verarbeitet
    def picture_handler(self):
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
        lower_blue = self.lower_blue_input
        print(lower_blue)
        upper_blue = self.upper_blue_input
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
        if alpha < 0:
            alpha = 90 + alpha + 90
        if alpha2 < 0:
            alpha2 = 90 + alpha2 + 90

        diffangle = 90 + (alpha2-alpha) # vielleicht muss mit Betrag (Vorzeichen) bearbeitet werden
        print(f"Diffwinkel   {diffangle}")

        return int(diffangle)

    def save_picture(self, img, diffangle):
        #picture_index = self.index #??
        int_angle = int(diffangle)
        picture_path = f"/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_{self.index}_{int_angle}.jpg"
        cv2.imwrite(picture_path, img)
        print(self.index)
        self.index += 1
            
        
        
        
    def video_handler(self):
        #angepasst mit while true
        #while True:
            video_stream = self.camera.get_frame()
            #img = video_stream
            video_small = cv2.resize(video_stream, None, fx=0.25, fy=0.25) # 50% prozent vom originalen Bild, fx, fy als parameter
            video_cropped = video_small.copy()[40:90, :, :] #40:90 und 20:150 als variablen oder einstellbar
            video_hsv = cv2.cvtColor(video_cropped,cv2.COLOR_BGR2HSV)
            lower_blue = self.lower_blue_input
            upper_blue = self.upper_blue_input
            video_filtered = cv2.inRange(video_hsv, lower_blue, upper_blue)
            video_edges = cv2.Canny(video_filtered, 900, 1000)
            lines = cv2.HoughLinesP(video_edges,  1, np.pi / 180, threshold=30, minLineLength=25, maxLineGap=10)
            
            return lines
            #self.frame = self.camera.get_frame()
        
        
        
    def video_streams(self):
        while True:
            frame = self.frame
            # self.frame = cv2.resize(self.cam.get_frame(), None, fx=0.25, fy=0.25)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_blue = self.lower_blue_input
            upper_blue = self.upper_blue_input
            filtered = cv2.inRange(hsv, lower_blue, upper_blue)
            _, frame_as_jpeg = cv2.imencode(".jpeg", filtered)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

            yield frame_as_string
            
            self.frame = self.camera.get_frame()
            
            
    
    def video_streams_Canny(self):
        while True:
            canny = self.frame
            # self.frame = cv2.resize(self.cam.get_frame(), None, fx=0.25, fy=0.25)
            hsv = cv2.cvtColor(canny, cv2.COLOR_BGR2HSV)
            lower_blue = self.lower_blue_input
            upper_blue = self.upper_blue_input
            filtered = cv2.inRange(hsv, lower_blue, upper_blue)
            video_edges = cv2.Canny(filtered, 900, 1000)
            _, frame_as_jpeg = cv2.imencode(".jpeg", video_edges)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

            yield frame_as_string
            # self.frame = self.camera.get_frame()
            
    def video_streams_lines(self): 
        while True: 
            video_stream = self.frame
            video_small = cv2.resize(video_stream, None, fx=0.25, fy=0.25) # 50% prozent vom originalen Bild, fx, fy als parameter
            video_cropped = video_small.copy()[40:90, :, :] #40:90 und 20:150 als variablen oder einstellbar
            video_hsv = cv2.cvtColor(video_cropped,cv2.COLOR_BGR2HSV)
            lower_blue = self.lower_blue_input
            upper_blue = self.upper_blue_input
            video_filtered = cv2.inRange(video_hsv, lower_blue, upper_blue)
            video_edges = cv2.Canny(video_filtered, 50, 100)
            lines = cv2.HoughLinesP(video_edges,  1, np.pi / 180, threshold=30, minLineLength=15, maxLineGap=10)
            video_line = video_cropped.copy()    
            angel = self.angle_calc(lines)
            video_line = cv2.putText(video_line, str(angel), (10,10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(video_line, (x1, y1), (x2, y2), (0, 255, 0), 1)
            _, frame_as_jpeg = cv2.imencode(".jpeg", video_line)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

            yield frame_as_string
            # self.frame = self.camera.get_frame()

# funktioniert noch nicht
    def start_mode(self, mode: int):
        if mode == 1:
            self._modus1()
        elif mode == 2:
            self._modus2()
        else:
            print(f"Unbekannter Modus")

        
# funktioniert noch nicht
    def _modus1(self):
        self.farb_config()
        self.drive(30,90)
        i = 0
        while i < 260:
            lines, img = self.video_handler()
            diffangle = self.angle_calc(lines)
            self.drive(new_angle=diffangle)
            self.save_picture(img, diffangle)
            #time.sleep(0.2)
            i +=1
        self.stop()
        print("Die Fahrt ist beendet")
        
        
        
        
        
        
        
   
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
    cam_car.farb_config()
    cam_car.drive(30,90)
    i = 0
    while i < 250:
        lines = cam_car.video_handler()
        print(lines)
        diffangle = cam_car.angle_calc(lines)        
        cam_car.drive(new_angle=diffangle)
        #cam_car.save_picture(img, diffangle)
        #time.sleep(0.1)
        i +=1 
    cam_car.stop()


    # lines, img = cam_car.picture_handler()
    # cam_car.angle_calc(lines)