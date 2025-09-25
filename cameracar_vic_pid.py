from basecar import BaseCar
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera
import time
import json
import numpy as np
import cv2
import tflite_runtime.interpreter as tflite

class CameraCar(BaseCar):

    def __init__(self, front, back, camera, values_to_log):
        super().__init__(front, back, values_to_log)
        self.camera = camera
        self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)
        self.index = 0
        self.upper_blue_input = np.array([90, 60, 60])
        self.lower_blue_input = np.array([130, 255, 255])
        self.hsv = np.zeros_like(self.frame)[:,:,0]
        self.canny = np.zeros_like(self.frame)[:,:,0]
        self.is_driving = False
        self.current_speed = 0            # velocidad actual usada por drive()
        self.CNN_active = False

        # Steering state used by PID (medido / aplicado)
        self.current_steering = 90        # empieza recto
        self.steering_angle = 90

        # PID defaults (puedes ajustarlos)
        self.kp = 0.8
        self.ki = 0.01
        self.kd = 0.1
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = time.time()
        self.pid_integral_limit = 100.0   # evita integral windup

        # límites del servo / ángulo
        self.steering_min = 45
        self.steering_max = 135

        self.angle_calc_CNN_ini("/home/pi/Camp2Code/C2C-PP02/C2C_PP_02/live_model_Vic_alle_fp32.tflite")
        print("CameraCar erzeugt (con PID)")

    # ---------- configuración PID ----------
    def set_pid_gains(self, kp=None, ki=None, kd=None):
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd

    def reset_pid(self):
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = time.time()

    def pid_control(self, setpoint):
        """
        setpoint: ángulo objetivo (por ejemplo, salida de angle_calc o CNN)
        devuelve: nuevo ángulo a aplicar (enteros entre steering_min y steering_max)
        """

        now = time.time()
        dt = now - self.pid_last_time
        if dt <= 0.0:
            dt = 1e-3

        # error = objetivo - medición
        error = float(setpoint) - float(self.current_steering)

        # integral con anti-windup
        self.pid_integral += error * dt
        # clamp integral
        if self.pid_integral > self.pid_integral_limit:
            self.pid_integral = self.pid_integral_limit
        elif self.pid_integral < -self.pid_integral_limit:
            self.pid_integral = -self.pid_integral_limit

        # derivada
        derivative = (error - self.pid_prev_error) / dt

        # salida PID (delta sobre la posición actual)
        control = (self.kp * error) + (self.ki * self.pid_integral) + (self.kd * derivative)

        # aplicar control sobre la posición actual
        new_angle = self.current_steering + control

        # saturar al rango físico del servo
        if new_angle < self.steering_min:
            new_angle = self.steering_min
        elif new_angle > self.steering_max:
            new_angle = self.steering_max

        # actualizar estado PID
        self.pid_prev_error = error
        self.pid_last_time = now

        # actualizar steering "medido" (simulado como lo que aplicamos)
        # (Si tu BaseCar provee feedback real del servo, reemplaza esto)
        self.current_steering = new_angle

        return int(round(new_angle))

    # ---------- resto de tus métodos (sin cambios funcionales importantes) ----------
    def farb_config(self):
        try:
            with open("config.json", "r") as ff:
                data = json.load(ff)
        except (FileNotFoundError, json.JSONDecodeError):
            print("Keine geeignete Datei config.json gefunden! Default-Werte werden gesetzt.")
            self.upper_blue_input = np.array([90, 60, 60])
            self.lower_blue_input = np.array([130, 255, 255])
        else:
            upper_blue_input = data.get("upper_blue_input", [90, 60, 60])
            lower_blue_input = data.get("lower_blue_input", [130, 255, 255])
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

    def angle_calc(self, lines, x_coord_left=55, x_coord_right=85):
        # Tu implementación existente (la pegué sin cambios)
        rechts = []
        links = []
        if lines is not None:
            for line in lines:
                xEnd, yEnd, xStart, yStart = line[0]
                if xEnd < x_coord_left:
                    links.append(line[0])
                elif xEnd > x_coord_right:
                    rechts.append(line[0])

        if len(rechts) == 1:
            rdurch = rechts
            if rdurch[0][2]-rdurch[0][0] != 0:
                alpha2 = np.arctan((rdurch[0][3]-rdurch[0][1])/(rdurch[0][2]-rdurch[0][0]))
                alpha2 = np.degrees(alpha2)
            else:
                alpha2 = 60
        elif len(rechts) >= 2:
            rdurch = np.mean(rechts, axis=0, keepdims=False)
            if rdurch[2]-rdurch[0] != 0:
                alpha2 = np.arctan((rdurch[3]-rdurch[1])/(rdurch[2]-rdurch[0]))
                alpha2 = np.degrees(alpha2)
            else:
                alpha2 = 60
        else:
            alpha2 = 60

        if len(links) == 1:
            ldurch = links
            if ldurch[0][2]-ldurch[0][0] != 0:
                alpha = np.arctan((ldurch[0][3]-ldurch[0][1])/(ldurch[0][2]-ldurch[0][0]))
                alpha = abs(np.degrees(alpha))
            else:
                alpha = 60
        elif len(links) >= 2:
            ldurch = np.mean(links, axis=0, keepdims=False)
            if ldurch[2]-ldurch[0] != 0:
                alpha = np.arctan((ldurch[3]-ldurch[1])/(ldurch[2]-ldurch[0]))
                alpha = abs(np.degrees(alpha))
            else:
                alpha = 60
        else:
            alpha = 60

        if alpha < 0:
            alpha = 180 + alpha 
        if alpha2 < 0:
            alpha2 = 180 + alpha2 

        if any(np.isnan([alpha, alpha2])):
            diffangle = 90
        else:
            diffangle = 90 + (alpha2 - alpha)

        return int(diffangle)

    def save_picture(self, frame, diffangle):
        int_angle = int(diffangle)
        if int_angle < 100:
            picture_path = f"/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_{self.index}_0{int_angle}.jpg"
        else:
            picture_path = f"/home/pi/Desktop/git/C2C_PP_02/pictures/Bild_{self.index}_{int_angle}.jpg"
        print("Bild gespeichert", self.index)
        cv2.imwrite(picture_path, frame)
        self.index += 1

    def video_streams(self):
        if self.CNN_active == False:
            while True:
                frame = self.frame
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_blue = self.lower_blue_input
                upper_blue = self.upper_blue_input
                self.hsv = cv2.inRange(hsv, lower_blue, upper_blue)
                _, frame_as_jpeg = cv2.imencode(".jpeg", self.hsv)
                frame_in_bytes = frame_as_jpeg.tobytes()
                frame_as_string = (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')
                yield frame_as_string

    def video_streams_Canny(self):
        if self.CNN_active == False:
            while True:
                self.canny = cv2.Canny(self.hsv, 100, 200)
                _, frame_as_jpeg = cv2.imencode(".jpeg", self.canny)
                frame_in_bytes = frame_as_jpeg.tobytes()
                frame_as_string = (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')
                yield frame_as_string

    def video_streams_lines(self):
        upper_boundary = 40
        while True:
            video_line_CNN = self.frame.copy()
            video_line_CNN = cv2.cvtColor(video_line_CNN, cv2.COLOR_BGR2RGB)
            video_line = video_line_CNN.copy()[upper_boundary:90, :, :]

            # lines se calcula a partir de la imagen Canny actual
            lines = cv2.HoughLinesP(self.canny[upper_boundary:90,:],  1, np.pi / 180, threshold=30, minLineLength=20, maxLineGap=10)

            # obtener ángulo objetivo (vision o CNN)
            if self.CNN_active:
                img = video_line_CNN
                img = cv2.resize(img, (128, 128)).astype(np.float32)/255
                img = np.expand_dims(img, axis=0)
                angel = self.angle_calc_CNN(img)
                # no cambie aquí: `angel` ya es el output del modelo
                print("CNN: ", angel)
            else:
                angel = self.angle_calc(lines)
                print("Angle Calc: ", angel)

            # --- PID compute ---
            target_angle = angel
            new_angle = self.pid_control(target_angle)
            self.steering_angle = new_angle

            # si estamos en modo conducción, enviar la orden al coche
            if self.is_driving:
                # aplica la velocidad actual almacenada y el nuevo ángulo
                try:
                    self.drive(self.current_speed, int(new_angle))
                except Exception as e:
                    print("drive() fallo:", e)

            # dibujar la información en el frame que servimos
            video_line = cv2.putText(video_line, str(new_angle), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(video_line, (x1, y1), (x2, y2), (0, 255, 0), 1)

            _, frame_as_jpeg = cv2.imencode(".jpeg", video_line)
            frame_in_bytes = frame_as_jpeg.tobytes()
            frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')
            yield frame_as_string

            # actualizar frame de cámara
            self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)

    def video_streams_lines_test(self):
        video_line_CNN = self.frame
        video_line_CNN = cv2.cvtColor(video_line_CNN, cv2.COLOR_BGR2RGB)
        img = video_line_CNN
        img = cv2.resize(img, (128, 128)).astype(np.float32)/255
        img = np.expand_dims(img, axis=0)
        angel = self.angle_calc_CNN(img)
        # aplicar PID también en este test
        new_angle = self.pid_control(angel)
        self.steering_angle = new_angle
        print("CNN (test): ", angel, "-> PID ->", new_angle)
        self.frame = cv2.resize(self.camera.get_frame(), None, fx=0.25, fy=0.25)

    def start_mode(self, mode: int):
        if mode == 1:
            self._modus1()
        elif mode == 2:
            self._modus2()
        else:
            print(f"Unbekannter Modus")

    def _modus1(self):
        self.farb_config()
        print("Die Fhrt beginnt")
        # ejemplo: poner a conducir con velocidad 30 y activar bandera is_driving
        self.current_speed = 30
        self.is_driving = True
        # reset PID cuando arrancamos
        self.reset_pid()
        # enviar una orden inicial para centrar
        self.drive(self.current_speed, int(self.current_steering))
