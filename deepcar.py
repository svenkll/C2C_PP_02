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

class DeepCar(BaseCar):

    def __init__(self, front, back, camera, values_to_log):
        super().__init__(front, back, values_to_log)
        self.camera = camera
        self.frame1 = self.camera.get_frame()
        self.frame = cv2.resize(self.frame1, None, fx=0.25, fy=0.25)

        # self.frameCNN = np.zeros_like(self.frame)[:,:,:]
        self.angle_calc_CNN_ini("/home/pi/Desktop/git/C2C_PP_02/live_model_Vic_alle_fp32.tflite")
        self.hsv = np.zeros_like(self.frame)[:,:,0]
        self.canny = np.zeros_like(self.frame)[:,:,0]
        self.is_driving = False
        print("CameraCar erzeugt")
        

        
        
        
    def video_streams(self):
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
            
            self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)
    
    def angle_calc_CNN_ini(self, path):
        self.interpreter = tflite.Interpreter(model_path=path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def angle_calc_CNN(self, img):
        self.interpreter.set_tensor(self.input_details[0]['index'], img)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        angle = output_data[0]
        return angle
        
        
        
    

            
            
    
    def video_streams_Canny(self):
        while True:

            self.canny = cv2.Canny(self.hsv, 100, 200)
            _, frame_as_jpeg = cv2.imencode(".jpeg", self.canny)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

            yield frame_as_string

            
    def video_streams_lines(self): 
        while True: 
            video_line = self.frame.copy()
            img = video_line
            img1 = cv2.resize(img, (128, 128)).astype(np.float32)/255
            img2 = np.expand_dims(img1, axis=0)
            angel = self.angle_calc_CNN(img2)
            self.steering_angle = angel
            print("Lenkwinkel: ", angel)
            
            
            _, frame_as_jpeg = cv2.imencode(".jpeg", video_line)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')
            # print(len(frame_as_string))
            # print(angel)

            yield frame_as_string


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
        #self.drive(30, 90)
        print("Vor der Schleife")
        while self.is_driving:
            print("In der Schleife: ", self.steering_angle)
            #self.drive(new_angle=self.steering_angle)
            self.save_picture(self.frame, self.steering_angle)
            time.sleep(0.2)

        self.stop()
        print("Die Fahrt ist beendet")
        
