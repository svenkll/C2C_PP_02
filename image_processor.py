from basisklassen_cam import Camera
import numpy as np
import cv2

class Processor():
    
    def __init__(self):
        self.cam = Camera()
        self.frame = cv2.resize(self.cam.get_frame(), None, fx=0.25, fy=0.25)
        self.lower = np.array([90, 0, 0])
        self.upper = np.array([130, 255, 255])
        #self.cam.release()

    def stream_image(self):
        while True:
            frame = self.frame
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            filtered = cv2.inRange(hsv, self.lower, self.upper)
            _, frame_as_jpeg = cv2.imencode(".jpeg", filtered)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

            yield frame_as_string

            self.frame = cv2.resize(self.cam.get_frame(), None, fx=0.25, fy=0.25)
            #self.cam.release()
    def stream_image_gray(self):
        while True:
            frame = self.frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, frame_as_jpeg = cv2.imencode(".jpeg", gray)
            frame_in_bytes = frame_as_jpeg.tobytes()

            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')

            yield frame_as_string

            self.frame = cv2.resize(self.cam.get_frame(), None, fx=0.25, fy=0.25)