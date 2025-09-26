from basecar import BaseCar
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera
import time
import json
import numpy as np
import matplotlib.pylab as plt
import cv2
import time
import tflite_runtime.interpreter as tflite





import time
import cv2
import numpy as np
from basecar import BaseCar
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera

# ---------- PID helper ----------
class SimplePID:
    def __init__(self, kp=1.2, ki=0.0, kd=0.2, setpoint=90, output_limits=(0,180)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.output_min, self.output_max = output_limits
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def reset(self):
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def update(self, measurement):
        now = time.time()
        if self._last_time is None:
            dt = 1e-3
        else:
            dt = max(1e-3, now - self._last_time)
        error = self.setpoint - measurement
        self._integral += error * dt
        derivative = (error - self._last_error) / dt
        out = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        self._last_error = error
        self._last_time = now
        out = max(self.output_min, min(self.output_max, out))  # clamp
        return int(out)
# --------------------------------
class CameraCar(BaseCar):

    def __init__(self, front, back, camera, values_to_log):
        super().__init__(front, back, values_to_log)
        self.camera = camera
        #??? liefert Video oder bilder zurück
        self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)
        self.index = 0
        self.upper_blue_input = np.array([90, 60, 60])
        self.lower_blue_input = np.array([130, 255, 255])
        self.hsv = np.zeros_like(self.frame)[:,:,0]
        self.canny = np.zeros_like(self.frame)[:,:,0]
        self.is_driving = False
        self.CNN_active = False
        self.angle_calc_CNN_ini("/home/pi/Camp2Code/C2C-PP02/C2C_PP_02/live_model_Vic_fp32.tflite")  #Victor's folder, to change it ;)
        # upper_blue_input in die init für die slider bei Dash // funktionen anpassen auf self. upper
        # lower_blue_input in die init für die slider bei Dash
        self.pid = SimplePID(kp=1.4, ki=0.0, kd=0.25, setpoint=90, output_limits=(45,135))
        # tune kp/kd and output_limits for your hardware: 45..135 are examples
        print("CameraCar erzeugt")
        
        # store steering angle state
        self.steering_angle = 90
        
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
            
            
    def angle_calc_CNN_ini(self, path):
        self.interpreter = tflite.Interpreter(model_path=path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def angle_calc_CNN(self, img):
        self.interpreter.set_tensor(self.input_details[0]['index'], img)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        angle = int(output_data[0][0])
        return angle
        

# werden die Bilder verarbeitet
    # def picture_handler(self):
    #     self.index += 1
    #     img = self.camera.get_frame()
    #     self.camera.release()
    #     #muss noch angepasst werden, wegen Dateien Benennung Ziel: Bild0x_Winkel_Grad
    #     cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild.jpg", img)
    #     # cv2.imwrite("Bild.jpg", img)
    #     img_read = cv2.imread("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild.jpg")
    #     img_small = cv2.resize(img_read, None, fx=0.25, fy=0.25) # 50% prozent vom originalen Bild, fx, fy als parameter
    #     cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_small.jpg", img_small)
    #     print(img_small.shape)
    #     img_cropped = img_small.copy()[60:90, :, :] #40:90 und 20:150 als variablen oder einstellbar
    #     cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_cropped.jpg", img_cropped)
    #     hsv = cv2.cvtColor(img_cropped,cv2.COLOR_BGR2HSV)
    #     print(hsv.shape)
    #     lower_blue = self.lower_blue_input
    #     print(lower_blue)
    #     upper_blue = self.upper_blue_input
    #     print(upper_blue)
    #     img_filtered = cv2.inRange(hsv, lower_blue, upper_blue)
    #     img_edges = cv2.Canny(img_filtered, 900, 1000)
    #     lines = cv2.HoughLinesP(img_edges,  1, np.pi / 180, threshold=30, minLineLength=25, maxLineGap=10) # extern Input ?
    #     #zu entscheiden, welche angpasst Bild soll gespecihert werden
    #     cv2.imwrite("/home/pi/Desktop/git/C2C_PP_02/pictures_modified/Bild_angepasst.jpg", img_edges)
    #     print(lines)
    #     return lines, img
    #     #plt.imshow(img)

#  # lenkwinkel berechnen und zurückgeben       
#     def angle_calc(self, lines, x_coord_left=55, x_coord_right=85):
#         rechts = []
#         links = []
        
#         if lines is not None:
#             for line in lines:
#                 xEnd, yEnd, xStart, yStart = line[0]
                
#                 if xEnd < x_coord_left:
#                     #print("Links")
#                     links.append(line[0])
#                 elif xEnd > x_coord_right:
#                     #print("Rechts")
#                     rechts.append(line[0])

#         if len(rechts) == 1:
#             rdurch = rechts
#             if rdurch[0][2]-rdurch[0][0] != 0:
#             #print(rdurch)
#                 alpha2 = np.arctan((rdurch[0][3]-rdurch[0][1])/(rdurch[0][2]-rdurch[0][0]))
#                 alpha2 = np.degrees(alpha2)
#                 #print(f"Alpha2 {alpha2}")
#             else:
#                 alpha2 = 60 # muss noch überprüft werden
#         elif len(rechts) >= 2:
#             rdurch = np.mean(rechts, axis=0, keepdims=False) #muss noch parameter angepasst werden
#             #(rechts[0] + rechts[1])/2
#             #print(rdurch)
#             if rdurch[2]-rdurch[0] != 0:
#                 alpha2 = np.arctan((rdurch[3]-rdurch[1])/(rdurch[2]-rdurch[0]))
#                 alpha2 = np.degrees(alpha2)
#                 #print(f"Alpha2 {alpha2}")
#             else:
#                 alpha2 = 60 # muss noch überprüft werden
#         else:
#             #print("no right lane")
#             alpha2 = 60 # muss noch überprüft werden

#         #mit einer linie probieren
#         if len(links) == 1:
#             ldurch = links 
#             if ldurch[0][2]-ldurch[0][0] != 0:
#             #print(f"Ldurch {ldurch}")
#                 alpha = np.arctan((ldurch[0][3]-ldurch[0][1])/(ldurch[0][2]-ldurch[0][0]))
#                 alpha = abs(np.degrees(alpha))
#             #print(f"Alpha {alpha}")
#             else:
#             #print("no left lane")
#                 alpha = 60 # muss noch überprüft werden
#         elif len(links) >= 2:
#             ldurch = np.mean(links, axis=0, keepdims=False)
#             if ldurch[2]-ldurch[0] != 0:
#                 alpha = np.arctan((ldurch[3]-ldurch[1])/(ldurch[2]-ldurch[0]))
#                 alpha = abs(np.degrees(alpha))
#             #print(f"Alpha {alpha}")
#             else:
#             #print("no left lane")
#                 alpha = 60 # muss noch überprüft werden
#         else:
#             #print("no left lane")
#             alpha = 60 # muss noch überprüft werden

#         # Berechnung des Lenkwinkels
#         if alpha < 0:
#             alpha = 180 + alpha 
#         if alpha2 < 0:
#             alpha2 = 180 + alpha2 

#         if any(np.isnan([alpha, alpha2])):
#             diffangle = 90
#         else:
#             diffangle = 90 + (alpha2 - alpha) # vielleicht muss mit Betrag (Vorzeichen) bearbeitet werden
#         #print("Test:", alpha, alpha2, diffangle)
#         #print(f"Diffwinkel   {diffangle}")

#         return int(diffangle)


    def save_picture(self, frame, diffangle):
        
        int_angle = int(diffangle)
        if int_angle < 100:
            picture_path = f"/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_{self.index}_0{int_angle}.jpg"
        else:
            picture_path = f"/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_{self.index}_{int_angle}.jpg"
            
        print("Bild gespeichert", self.index)
        cv2.imwrite(picture_path, frame)
        #print(self.index)
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
            angel = self.angle_calc(lines)
            self.steering_angle = angel
            print("Angle Calc: ", angel)
            #return lines
            #self.frame = self.camera.get_frame()
        
        
    def video_streams(self):
        if self.CNN_active == False:
            while True:
                frame = self.frame
                # self.save_picture(frame, self.steering_angle)
                # self.frame = cv2.resize(self.cam.get_frame(), None, fx=0.25, fy=0.25)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_blue = self.lower_blue_input
                upper_blue = self.upper_blue_input
                self.hsv = cv2.inRange(hsv, lower_blue, upper_blue)
                _, frame_as_jpeg = cv2.imencode(".jpeg", self.hsv)
                frame_in_bytes = frame_as_jpeg.tobytes()

                frame_as_string = (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

                yield frame_as_string
                
                #self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)

    def video_streams_Canny(self):
        if self.CNN_active == False:
            while True:

                self.canny = cv2.Canny(self.hsv, 100, 200)
                _, frame_as_jpeg = cv2.imencode(".jpeg", self.canny)
                frame_in_bytes = frame_as_jpeg.tobytes()

                frame_as_string = (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

                yield frame_as_string

            
    def video_streams_lines_test(self):
        # get fresh frame immediately
        frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)
        video_line_CNN = frame.copy()
        video_line_CNN = cv2.cvtColor(video_line_CNN, cv2.COLOR_BGR2RGB)

        img = cv2.resize(video_line_CNN, (128, 128)).astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        t0 = time.time()
        angel = self.angle_calc_CNN(img)
        infer_dt = time.time() - t0

        try:
            cnn_angle = float(angel)
        except:
            cnn_angle = 90.0

        mapped_servo_angle = int(max(45, min(135, cnn_angle)))  
        measurement = getattr(self, "steering_angle", 90)

        self.pid.setpoint = mapped_servo_angle
        servo_command = self.pid.update(measurement)

        self.drive(30, servo_command)   # speed=30, steering=servo_command
        self.steering_angle = servo_command

        print(f"CNN raw: {cnn_angle} -> mapped: {mapped_servo_angle} | PID out: {servo_command} | inf {infer_dt*1000:.0f}ms")

        self.frame = frame


    def video_streams_lines_test(self): 
        
        #while True: 
        video_line_CNN = self.frame #40:90 und 20:150 als variablen oder einstellbar
        video_line_CNN = cv2.cvtColor(video_line_CNN, cv2.COLOR_BGR2RGB)
    

        img = video_line_CNN
        img = cv2.resize(img, (128, 128)).astype(np.float32)/255
        img = np.expand_dims(img, axis=0)
        angel = self.angle_calc_CNN(img)
        self.steering_angle = angel
        print("CNN: ", angel)
        self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)




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
        print("Die Fhrt beginnt")
        self.drive(30, 90)
        # print("Vor der Schleife")
        # while self.is_driving:
            # print("In der Schleife: ", self.steering_angle)
            #self.drive(new_angle=self.steering_angle)
            # self.save_picture(self.frame, self.steering_angle)
            # time.sleep(0.2)

        # self.stop()
        # print("Die Fahrt ist beendet")
        
        

if __name__ == "__main__":

    front = FrontWheels()
    back = BackWheels()
    cam = Camera(devicenumber = 0,
                buffersize = 1,
                skip_frame = 0,
                height = 480,
                width = 640,
                flip = True,
                )



    cam_car = CameraCar(front,back,cam, [])
    #cam_car.farb_config()
    cam_car.drive(30,90)
    #lines, frame = cam_car.video_streams_lines()
    i = 0
    while i <= 300:
        cam_car.video_streams_lines_test()
        i += 1
        
    cam_car.stop()
        # print(lines)
        # diffangle = cam_car.angle_calc(lines)        
        # cam_car.drive(new_angle=diffangle)
        #cam_car.save_picture(img, diffangle)
        #time.sleep(0.1)


    # lines, img = cam_car.picture_handler()
    # cam_car.angle_calc(lines)