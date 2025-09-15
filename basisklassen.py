# coding=utf-8
#!/usr/bin/env python
"""
    Information: Module mit Basisklassen für Projekt RPiCar
    File name: basisklassen.py
    Author: Robert Heise (FIDA)
    Date created: 11/10/2021
    Date last modified: 23/10/2021
    Python Version: 3.7
    Usage: Basisklassen U4I/FIDA "Autonomens Fahren" mit Sunfounder PiCar-S Projektphase 1
"""


import click
import time
import numpy as np
import math
import RPi.GPIO as GPIO
import smbus


class Ultrasonic(object):
    """Eine Klasse für das SunFounder Ultraschall-Module

    Attributes:
        CHANNEL (int): GPIO-Kanal des Ultraschall-Moduls. GPIO entsprechend des Sunfounder Setups gewählt.
        preparation_time (float): Wartezeit in Millisekunden vor Aussenden des Ultraschall-Impulses.
        impuls_length (float): Länge des Ultraschall-Impulses.
        timeout (float): Wartezeit bis zum Abruch der Messung in Form der maximale Messdauer.
    Methoden:
        distance[int]: Distanz in cm für eine Einzelmessung. Negative Werte für Fehler.
        stop: Abschalten
    """

    CHANNEL = 20  # GPIO entsprechend des Sunfounder Setups

    def __init__(
        self,
        preparation_time: float = 0.01,
        impuls_length: float = 0.00001,
        timeout: float = 0.05,
    ) -> None:
        """Konstruktors der Klasse Ultrasonic

        Args:
            preparation_time (float): Wartezeit in Millisekunden vor Aussenden des Ultraschall-Impulses. Defaults to 0.01.
            impuls_length (float): Länge des Ultraschall-Impulses. Defaults to 0.00001.
            timeout (float): Wartezeit bis zum Abruch der Messung in Form der maximale Messdauer. Defaults to 0.05.
        """
        self.preparation_time = preparation_time
        self.implus_length = impuls_length
        self.timeout = timeout
        GPIO.setmode(GPIO.BCM)

    def distance(self) -> int:
        """Returns distance in cm

        Returns:
            [int]: Distanz in cm für eine Einzelmessung. Negative Werte für Fehler.
        """
        pulse_end = 0
        pulse_start = 0
        GPIO.setup(self.CHANNEL, GPIO.OUT)
        GPIO.output(self.CHANNEL, False)
        time.sleep(self.preparation_time)
        GPIO.output(self.CHANNEL, True)
        time.sleep(self.implus_length)
        GPIO.output(self.CHANNEL, False)
        GPIO.setup(self.CHANNEL, GPIO.IN)

        timeout_start = time.time()
        while GPIO.input(self.CHANNEL) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while GPIO.input(self.CHANNEL) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -2

        if pulse_start != 0 and pulse_end != 0:
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 100 * 343.0 / 2
            distance = int(distance)
            if distance >= 0:
                return distance
            else:
                return -3
        else:
            return -4

    def stop(self) -> None:
        """Setzt GPIO (channel) des Raspberry auf False und beendet eventuell auftretende Geräusche des Sensor (Abschalten)"""
        # Setzt GPIO als Output mit dem Wert False
        GPIO.setup(self.CHANNEL, GPIO.OUT)
        GPIO.output(self.CHANNEL, False)

    def test(self) -> None:
        """Prints 10 measurements in 5 seconds, negative numbers for errors which can appear for large distances"""
        for i in range(10):
            distance = self.distance()
            if distance < 0:
                unit = "Error"
            else:
                unit = "cm"
            print("{} : {} {}".format(i, distance, unit))
            time.sleep(0.5)


class Infrared(object):
    """Eine Klasse für das SunFounder Infrarot-Modul

    Attributes:
        ADDRESS (int): Adresse des Infrarot-Moduls.
        references (list): Liste von Float, die als Referenzwerte dienen.
        bus (smbus): Bus des Infrarot-Moduls.
    """

    ADDRESS = 0x11

    def __init__(self, references: list = [300, 300, 300, 300, 300]) -> None:
        """Konstruktors der Klasse Infrared

        Args:
            references (list, optional): Liste von Float, die als Referenzwerte dienen. Defaults to [300, 300, 300, 300, 300].
        """
        self._bus = smbus.SMBus(1)
        self._references = references

    def _read_raw(self) -> list:
        """Liest den Wert des Infrarot-Moduls als raw ein.

        Returns:
            [list]: Liste von Bytes, die als raw eingelesen wurden.
        """
        for _ in range(0, 5):
            try:
                raw_result = self._bus.read_i2c_block_data(self.ADDRESS, 0, 10)
                Connection_OK = True
                break
            except:
                Connection_OK = False
        if Connection_OK:
            return raw_result
        else:
            print("Error accessing Infrared Sensor at address".format(self.ADDRESS))
            return False

    def read_analog(self, trys: int = 5) -> list:
        """Liest den Wert des Infrarot-Moduls als analog ein.

        Args:
            trys (int): Anzahl der Versuche, die die Verbindung zum Modul aufzubauen. Defaults to 5.

        Raises:
            IOError: Wenn die Verbindung zum Modul nicht aufgebaut werden kann.

        Returns:
            [list]: Liste von Bytes der Messung der einzelnen Sensoren, die als analog eingelesen wurden.
        """
        for _ in range(trys):
            raw_result = self._read_raw()
            if raw_result:
                analog_result = [0, 0, 0, 0, 0]
                for i in range(0, 5):
                    high_byte = raw_result[i * 2] << 8
                    low_byte = raw_result[i * 2 + 1]
                    analog_result[i] = high_byte + low_byte
                    if analog_result[i] > 1024:
                        continue
                return analog_result  # ,raw_result
        else:
            raise IOError("Line follower read error. Please check the wiring.")

    # TODO Could also be done in class car!
    def read_digital(self) -> list:
        """Liest den Wert des Infrarot-Moduls als digital ein.

        Returns:
            [list]: Liste der digitalisierten Messung der Sensoren unter Verwendung der Referenz als Schwellwert
        """
        analog = np.array(self.read_analog())
        digital = np.where(analog < self._references, 1, 0)
        return list(digital)

    def get_average(self, mount: int = 10) -> list:
        """Gibt den Mittelwert der Messungen zurück.

        Args:
            mount (int): Anzahl der Messungen, die durchgeführt werden. Defaults to 10.

        Returns:
            [float]: Liste der Messung als Mittelwert von 'mount' Einzelmessungen
        """
        single_reads = []
        for _ in range(mount):
            single_reads.append(self.read_analog())
        single_reads = np.array(single_reads)
        return list(single_reads.mean(axis=0))

    def set_references(self, ref: list) -> None:
        """Setzt die Referenz 'ref' als Liste von Float

        Args:
            ref (list): Liste von Float, die als Referenzwerte dienen.
        """
        self._references = ref

    def cali_references(self) -> None:
        """Aufnahme der Referenz"""
        input("Place on background:")
        background = self.get_average(100)
        print("measured background:", background)
        input("Place on line:")
        line = self.get_average(100)
        print("measured line:", line)
        self._references = (np.array(line) + np.array(background)) / 2
        print("Reference:", self._references)

    def test(self) -> None:
        """Testet die Verbindung zum Infrarot-Modul. Prints 10 measurements in 5 seconds."""
        for i in range(10):
            data = self.read_analog()
            print("{} : {}".format(i, data))
            time.sleep(0.5)


class FrontWheels(object):
    """Eine Klasse für die SunFounder Lenkung der Vorderräder. Sie erlaubt maximale Lenkungswinkel von 45°

    Attributes:
        FRONT_LEFT_WHEEL_ADDRESS (int): Adresse des Vorderrad-Moduls für die linke Lenkung.
        MAX_ANGLE (int): Maximale Lenkungswinkel.
        BUS (smbus): Bus des Vorderrad-Moduls.
        straight_angle (int): Winkel, der als geradeaus gesetzt wird.
        turning_offset (int): Offset, der zur Berechnung des Winkels verwendet wird.
    Methoden:
        turn: Setzen des Lenkwinkels (90 = geradeaus)
        get_angles: Maximale Lenkwinkel
    """

    FRONT_WHEEL_CHANNEL = 0  # from Sunfounder
    MAX_TURNING_ANGLE = 45  # in order to avoid damage
    BUS_NUMBER = 1  # from Sunfounder

    def __init__(self, turning_offset: int = 0) -> None:
        """setup channels and basic stuff

        Args:
            turning_offset (int): Offset, der zur Berechnung des Winkels verwendet wird. Defaults to 0.
        """
        self._straight_angle = 90
        self._turning_offset = turning_offset
        self._servo = Servo(
            self.FRONT_WHEEL_CHANNEL,
            bus_number=self.BUS_NUMBER,
            offset=self._turning_offset,
        )
        self._servo.setup()  # Init the class with bus_number and address
        self._turning_max(self.MAX_TURNING_ANGLE)

    def _turning_max(self, angle: int) -> None:
        """Setzt den maximalen und minimalen Lenkungswinkel auf 'angle'.

        Args:
            angle (int): Maximaler Lenkungswinkel.
        """
        self._turning_max = angle
        self._min_angle = self._straight_angle - angle
        self._max_angle = self._straight_angle + angle
        self._angles = {
            "left": self._min_angle,
            "straight": self._straight_angle,
            "right": self._max_angle,
        }

    def turn(self, angle: int) -> int:
        """Turn the front wheels to the giving angle. Setzt Lenkungswinkel 'angle' als Int im Interval von 45-135° (90° geradeaus).

        Args:
            angle (int): Lenkwinkel, der gesetzt werden soll.

        Returns:
            int: gesetzter Lenkwinkel
        """
        if angle == None:
            angle = self._straight_angle
        if angle < self._angles["left"]:
            angle = self._angles["left"]
        if angle > self._angles["right"]:
            angle = self._angles["right"]
        self._servo.write(angle)
        return angle

    def get_angles(self) -> dict:
        """Gibt die maximalen Lenkungswinkel zurück.

        Returns:
            [dict]: Dictionary der maximalen Lenkungswinkel
        """
        return self._angles

    def test(self) -> None:
        """Testet die Verbindung zum Vorderrad-Modul. Prints 10 measurements in 5 seconds."""
        self.turn(90)
        print("angle : {}".format(90))
        for a in range(self._angles["left"], self._angles["right"] + 1, 5):
            time.sleep(0.1)
            self.turn(a)
            print("angle : {}".format(a))
        time.sleep(0.1)
        self.turn(90)
        print("angle : {}".format(90))


# import TB6612
# import PCA9685
class BackWheels(object):
    """
    Eine Klasse für den SunFounder Hinterradantrieb.

    Attributes:
        MOTOR_A (int): Adresse des Motor-Moduls für eine Seite des Hinterrads.
        MOTOR_B (int): Adresse des Motor-Moduls für die andere Seite des Hinterrads.
        PWM_A (int): Adresse des PWM-Moduls für eine Seite des Hinterrads.
        PWM_B (int): Adresse des PWM-Moduls für die andere Seite des Hinterrads.
        BUS_NUMBER (int): Bus-Nummer des Motor-Moduls.
        forward_A (int): Drehrichtung für Motor A.
        forward_B (int): Drehrichtung für Motor B.
    Methoden:
        speed (int): Setzen/Abfrage der Geschwindigkeit der Motoren.
        forward: vorwärts fahren
        backward: rückwärts fahren
    """

    MOTOR_A = 17
    MOTOR_B = 27

    PWM_A = 4
    PWM_B = 5

    BUS_NUMBER = 1

    def __init__(self, forward_A: int = 0, forward_B: int = 0) -> None:
        """Init the direction channel and pwm channel

        Args:
            forward_A (int):  0,1 (Konfiguration der Drehrichtung eines Motors). Defaults to 0.
            forward_B (int):  0,1 (Konfiguration der Drehrichtung des anderen Motors). Defaults to 0.
        """
        self.forward_A = forward_A
        self.forward_B = forward_B

        self.left_wheel = Motor(self.MOTOR_A, offset=self.forward_A)
        self.right_wheel = Motor(self.MOTOR_B, offset=self.forward_B)
        self.pwm = PWM(bus_number=self.BUS_NUMBER)

        def _set_a_pwm(value) -> None:
            """Setzt die PWM-Werte für Motor A.

            Args:
                value (int): PWM-Wert für Motor A.
            """
            pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
            self.pwm.write(self.PWM_A, 0, pulse_wide)

        def _set_b_pwm(value) -> None:
            """Setzt die PWM-Werte für Motor B.

            Args:
                value (int): PWM-Wert für Motor B.
            """
            pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
            self.pwm.write(self.PWM_B, 0, pulse_wide)

        self.left_wheel.pwm = _set_a_pwm
        self.right_wheel.pwm = _set_b_pwm
        self._speed = 0

    def forward(self) -> None:
        """Setzt Fahrmodus auf vorwärts. Move both wheels forward"""
        self.left_wheel.forward()
        self.right_wheel.forward()

    def backward(self) -> None:
        """Setzt Fahrmodus auf rückwärts. Move both wheels backward"""
        self.left_wheel.backward()
        self.right_wheel.backward()

    def stop(self) -> None:
        """Setzt die Geschwindigkeit auf 0."""
        self.speed = 0

    @property
    def speed(self):
        """Gets the speed. 0-100. 0 is stop. 100 is max speed.

        Verwendung als getter: Int=BW.speed

        Returns:
            int: Geschwindigkeit der Motoren.
        """
        return self._speed

    @speed.setter
    def speed(self, speed):
        """Sets the speed. 0-100. 0 is stop. 100 is max speed. It also sets the speed of the left and right back wheels.

        Verwendung als setter: BW.speed=Int

        Args:
            speed (int): Geschwindigkeit der Motoren.
        """
        self._speed = speed
        self.left_wheel.speed = self._speed
        self.right_wheel.speed = self._speed

    def test(self, t: int = 1) -> None:
        """Testet das setzten der Geschwindigkeit. Testet forwärts und rückwärts Fahren für t Sekunden.

        Args:
            t (int): Zeit in Sekunden, die gefahren werden soll. Defaults to 1.
        """
        self.speed = 30
        self.forward()
        print("forward speed : {}".format(self.speed))

        time.sleep(t)
        self.speed = 40
        print("forward speed : {}".format(self.speed))

        time.sleep(t)
        self.speed = 20
        print("forward speed : {}".format(self.speed))

        time.sleep(t)
        self.stop()
        print("stop speed : {}".format(self.speed))
        time.sleep(t * 2)
        self.speed = 20
        print("forward speed : {}".format(self.speed))

        time.sleep(t)
        self.backward()
        print("now backward")
        print("backward speed : {}".format(self.speed))

        time.sleep(t)
        self.speed = 0
        print("stop speed : {}".format(self.speed))


"""
The following lines contain the classes defined in Servo.py, TB6612.py, and PCA9698.py.
These classes could be defined in one single file.
"""

"""
taken from
**********************************************************************
* Filename    : Servo.py
* Description : Driver module for servo, with PCA9685
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Cavon    2016-09-13    New release
*               Cavon    2016-09-21    Change channel from 1 to all
**********************************************************************
"""


class Servo(object):
    """Servo driver class"""

    _MIN_PULSE_WIDTH = 600
    _MAX_PULSE_WIDTH = 2400
    _DEFAULT_PULSE_WIDTH = 1500
    _FREQUENCY = 60

    _DEBUG = False
    _DEBUG_INFO = 'DEBUG "Servo.py":'

    def __init__(self, channel, offset=0, lock=True, bus_number=1, address=0x40):
        """ Init a servo on specific channel, this offset """
        if channel < 0 or channel > 16:
            raise ValueError('Servo channel "{0}" is not in (0, 15).'.format(channel))
        self._debug_("Debug on")
        self.channel = channel
        self.offset = offset
        self.lock = lock

        self.pwm = PWM(bus_number=bus_number, address=address)
        self.frequency = self._FREQUENCY
        self.write(90)

    def _debug_(self, message):
        if self._DEBUG:
            print(self._DEBUG_INFO, message)

    def setup(self):
        self.pwm.setup()

    def _angle_to_analog(self, angle):
        """ Calculate 12-bit analog value from giving angle """
        pulse_wide = self.pwm.map(
            angle, 0, 180, self._MIN_PULSE_WIDTH, self._MAX_PULSE_WIDTH
        )
        analog_value = int(float(pulse_wide) / 1000000 * self.frequency * 4096)
        self._debug_("Angle %d equals Analog_value %d" % (angle, analog_value))
        return analog_value

    @property
    def frequency(self):
        return self._frequency

    @frequency.setter
    def frequency(self, value):
        self._frequency = value
        self.pwm.frequency = value

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, value):
        """ Set offset for much user-friendly """
        self._offset = value
        self._debug_("Set offset to %d" % self.offset)

    def write(self, angle):
        """ Turn the servo with giving angle. """
        if self.lock:
            if angle > 180:
                angle = 180
            if angle < 0:
                angle = 0
        else:
            if angle < 0 or angle > 180:
                raise ValueError(
                    'Servo "{0}" turn angle "{1}" is not in (0, 180).'.format(
                        self.channel, angle
                    )
                )
        val = self._angle_to_analog(angle)
        val += self.offset
        self.pwm.write(self.channel, 0, val)
        self._debug_("Turn angle = %d" % angle)

    @property
    def debug(self):
        return self._DEBUG

    @debug.setter
    def debug(self, debug):
        """ Set if debug information shows """
        if debug in (True, False):
            self._DEBUG = debug
        else:
            raise ValueError(
                'debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(
                    debug
                )
            )

        if self._DEBUG:
            print(self._DEBUG_INFO, "Set debug on")
        else:
            print(self._DEBUG_INFO, "Set debug off")


"""
taken from
**********************************************************************
* Filename    : TB6612.py
* Description : A driver module for TB6612
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Cavon    2016-09-23    New release
**********************************************************************
"""
# import RPi.GPIO as GPIO


class Motor(object):
    """Motor driver class
    Set direction_channel to the GPIO channel which connect to MA,
    Set motor_B to the GPIO channel which connect to MB,
    Both GPIO channel use BCM numbering;
    Set pwm_channel to the PWM channel which connect to PWMA,
    Set pwm_B to the PWM channel which connect to PWMB;
    PWM channel using PCA9685, Set pwm_address to your address, if is not 0x40
    Set debug to True to print out debug informations.
    """

    _DEBUG = False
    _DEBUG_INFO = 'DEBUG "TB6612.py":'

    def __init__(self, direction_channel, pwm=None, offset=True):
        """Init a motor on giving dir. channel and PWM channel."""
        self._debug_("Debug on")
        self.direction_channel = direction_channel
        self._pwm = pwm
        self._offset = offset
        self.forward_offset = self._offset

        self.backward_offset = not self.forward_offset
        self._speed = 0

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self._debug_("setup motor direction channel at %s" % direction_channel)
        self._debug_(
            "setup motor pwm channel"
        )  # self._debug_('setup motor pwm channel as %s ' % self._pwm.__name__)
        GPIO.setup(self.direction_channel, GPIO.OUT)

    def _debug_(self, message):
        if self._DEBUG:
            print(self._DEBUG_INFO, message)

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        """ Set Speed with giving value """
        if speed not in range(0, 101):
            raise ValueError('speed ranges fron 0 to 100, not "{0}"'.format(speed))
        if not callable(self._pwm):
            raise ValueError(
                "pwm is not callable, please set Motor.pwm to a pwm control function with only 1 veriable speed"
            )
        self._debug_("Set speed to: %s" % speed)
        self._speed = speed
        self._pwm(self._speed)

    def forward(self):
        """ Set the motor direction to forward """
        GPIO.output(self.direction_channel, self.forward_offset)
        self.speed = self._speed
        self._debug_("Motor moving forward (%s)" % str(self.forward_offset))

    def backward(self):
        """ Set the motor direction to backward """
        GPIO.output(self.direction_channel, self.backward_offset)
        self.speed = self._speed
        self._debug_("Motor moving backward (%s)" % str(self.backward_offset))

    def stop(self):
        """ Stop the motor by giving a 0 speed """
        self._debug_("Motor stop")
        self.speed = 0

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, value):
        """ Set offset for much user-friendly """
        if value not in (True, False):
            raise ValueError('offset value must be Bool value, not"{0}"'.format(value))
        self.forward_offset = value
        self.backward_offset = not self.forward_offset
        self._debug_("Set offset to %d" % self._offset)

    @property
    def debug(self, debug):
        return self._DEBUG

    @debug.setter
    def debug(self, debug):
        """ Set if debug information shows """
        if debug in (True, False):
            self._DEBUG = debug
        else:
            raise ValueError(
                'debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(
                    debug
                )
            )

        if self._DEBUG:
            print(self._DEBUG_INFO, "Set debug on")
        else:
            print(self._DEBUG_INFO, "Set debug off")

    @property
    def pwm(self):
        return self._pwm

    @pwm.setter
    def pwm(self, pwm):
        self._debug_("pwm set")
        self._pwm = pwm


"""
taken from
**********************************************************************
* Filename    : PCA9685.py
* Description : A driver module for PCA9685
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Version     : v2.0.0
**********************************************************************
"""
# import smbus
# import time
# import math


class PWM(object):
    """A PWM control class for PCA9685."""

    _MODE1 = 0x00
    _MODE2 = 0x01
    _SUBADR1 = 0x02
    _SUBADR2 = 0x03
    _SUBADR3 = 0x04
    _PRESCALE = 0xFE
    _LED0_ON_L = 0x06
    _LED0_ON_H = 0x07
    _LED0_OFF_L = 0x08
    _LED0_OFF_H = 0x09
    _ALL_LED_ON_L = 0xFA
    _ALL_LED_ON_H = 0xFB
    _ALL_LED_OFF_L = 0xFC
    _ALL_LED_OFF_H = 0xFD

    _RESTART = 0x80
    _SLEEP = 0x10
    _ALLCALL = 0x01
    _INVRT = 0x10
    _OUTDRV = 0x04

    _DEBUG = False
    _DEBUG_INFO = 'DEBUG "PCA9685.py":'

    def __init__(self, bus_number=1, address=0x40):
        self.address = address
        self.bus_number = bus_number
        self.bus = smbus.SMBus(self.bus_number)

    def _debug_(self, message):
        if self._DEBUG:
            print(self._DEBUG_INFO, message)

    def setup(self):
        """Init the class with bus_number and address"""
        self._debug_("Reseting PCA9685 MODE1 (without SLEEP) and MODE2")
        self.write_all_value(0, 0)
        self._write_byte_data(self._MODE2, self._OUTDRV)
        self._write_byte_data(self._MODE1, self._ALLCALL)
        time.sleep(0.005)

        mode1 = self._read_byte_data(self._MODE1)
        mode1 = mode1 & ~self._SLEEP
        self._write_byte_data(self._MODE1, mode1)
        time.sleep(0.005)
        self._frequency = 60

    def _write_byte_data(self, reg, value):
        """Write data to I2C with self.address"""
        self._debug_("Writing value %2X to %2X" % (value, reg))
        try:
            self.bus.write_byte_data(self.address, reg, value)
        except Exception as i:
            print(i)
            self._check_i2c()

    def _read_byte_data(self, reg):
        """Read data from I2C with self.address"""
        self._debug_("Reading value from %2X" % reg)
        try:
            results = self.bus.read_byte_data(self.address, reg)
            return results
        except Exception as i:
            print(i)
            self._check_i2c()

    def _run_command(self, cmd):
        import subprocess

        p = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        )
        result = p.stdout.read().decode("utf-8")
        status = p.poll()
        # print(result)
        # print(status)
        return status, result

    def _check_i2c(self):
        from os import listdir

        print("I2C bus number is: %s" % self.bus_number)
        print("Checking I2C device:")
        devices = listdir("/dev/")
        if "i2c-%d" % self.bus_number in devices:
            print("I2C device exist.")
        else:
            print(
                "Seems like I2C have not been set, run 'sudo raspi-config' to enable I2C"
            )
        cmd = "i2cdetect -y %s" % self.bus_number
        _, output = self._run_command(cmd)
        print("Your PCA9685 address is set to 0x%02X" % self.address)
        print("i2cdetect output:")
        print(output)
        outputs = output.split("\n")[1:]
        addresses = []
        for tmp_addresses in outputs:
            tmp_addresses = tmp_addresses.split(":")
            if len(tmp_addresses) < 2:
                continue
            else:
                tmp_addresses = tmp_addresses[1]
            tmp_addresses = tmp_addresses.strip().split(" ")
            for address in tmp_addresses:
                if address != "--":
                    addresses.append(address)
        print("Conneceted i2c device:")
        if addresses == []:
            print("None")
        else:
            for address in addresses:
                print("  0x%s" % address)
        if "%02X" % self.address in addresses:
            print(
                "Wierd, I2C device is connected, Try to run the program again, If problem stills, email this information to support@sunfounder.com"
            )
        else:
            print("Device is missing.")
            print(
                "Check the address or wiring of PCA9685 Server driver, or email this information to support@sunfounder.com"
            )
            quit()

    @property
    def frequency(self):
        return self._frequency

    @frequency.setter
    def frequency(self, freq):
        """Set PWM frequency"""
        self._debug_("Set frequency to %d" % freq)
        self._frequency = freq
        prescale_value = 25000000.0
        prescale_value /= 4096.0
        prescale_value /= float(freq)
        prescale_value -= 1.0
        self._debug_("Setting PWM frequency to %d Hz" % freq)
        self._debug_("Estimated pre-scale: %d" % prescale_value)
        prescale = math.floor(prescale_value + 0.5)
        self._debug_("Final pre-scale: %d" % prescale)

        old_mode = self._read_byte_data(self._MODE1)
        new_mode = (old_mode & 0x7F) | 0x10
        self._write_byte_data(self._MODE1, new_mode)
        self._write_byte_data(self._PRESCALE, int(math.floor(prescale)))
        self._write_byte_data(self._MODE1, old_mode)
        time.sleep(0.005)
        self._write_byte_data(self._MODE1, old_mode | 0x80)

    def write(self, channel, on, off):
        """Set on and off value on specific channel"""
        self._debug_('Set channel "%d" to value "%d"' % (channel, off))
        self._write_byte_data(self._LED0_ON_L + 4 * channel, on & 0xFF)
        self._write_byte_data(self._LED0_ON_H + 4 * channel, on >> 8)
        self._write_byte_data(self._LED0_OFF_L + 4 * channel, off & 0xFF)
        self._write_byte_data(self._LED0_OFF_H + 4 * channel, off >> 8)

    def write_all_value(self, on, off):
        """Set on and off value on all channel"""
        self._debug_('Set all channel to value "%d"' % (off))
        self._write_byte_data(self._ALL_LED_ON_L, on & 0xFF)
        self._write_byte_data(self._ALL_LED_ON_H, on >> 8)
        self._write_byte_data(self._ALL_LED_OFF_L, off & 0xFF)
        self._write_byte_data(self._ALL_LED_OFF_H, off >> 8)

    def map(self, x, in_min, in_max, out_min, out_max):
        """To map the value from arange to another"""
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    @property
    def debug(self):
        return self._DEBUG

    @debug.setter
    def debug(self, debug):
        """Set if debug information shows"""
        if debug in (True, False):
            self._DEBUG = debug
        else:
            raise ValueError(
                'debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(
                    debug
                )
            )

        if self._DEBUG:
            print(self._DEBUG_INFO, "Set debug on")
        else:
            print(self._DEBUG_INFO, "Set debug off")


@click.command()
@click.option(
    "--modus", "--m", type=int, default=None, help="Startet Test für Klasse direkt."
)
def main(modus):
    """
    Test der Basisklassen
    """
    print("-- DEMO BASISKLASSEN--------------------")
    modi = {
        0: "Ausrichtung der Servo der Lenkung",
        1: "Test Hinterräder - Antrieb / Klasse: Back_Wheels",
        2: "Test Vorderräder - Lenkung / Klasse: Front_Wheels",
        3: "Test Ultraschallmodul / Klasse: Ultrasonic",
        4: "Test Infrarotmodul / Klasse: Infrared",
    }

    if modus == None:
        print("--" * 20)
        print("Auswahl:")
        for m in modi.keys():
            print("{i} - {name}".format(i=m, name=modi[m]))
        print("--" * 20)

    while modus == None:
        modus = input("Wähle  (Andere Taste für Abbruch): ? ")
        if modus in ["1", "2", "3", "4"]:
            break
        else:
            modus = None
            print("Getroffene Auswahl nicht möglich.")
            quit()
    modus = int(modus)

    if modus == 0:
        print("Ausrichtung der Vorderräder")
        fw = FrontWheels()
        fw.turn(45)
        time.sleep(0.5)
        fw.turn(135)
        time.sleep(0.5)
        print(
            "Servo der Lenkung auf 90 Grad/geradeaus ausgerichtet! (CRTL-C zum beenden)"
        )
        while True:
            fw.turn(90)
            time.sleep(1)

    if modus == 1:
        x = input(
            "ACHTUNG! Das Auto wird ein Stück fahren!\n Dücken Sie ENTER zum Start."
        )
        print("Test Hinterräder")
        if x == "":
            bw = BackWheels()
            bw.test()
        else:
            print("Abruch.")

    if modus == 2:
        print("Test Vorderräder")
        fw = FrontWheels()
        fw.test()

    if modus == 3:
        print("Test Ultrasonic")
        usm = Ultrasonic()
        usm.test()

    if modus == 4:
        print("Test Infrared")
        irm = Infrared()
        irm.test()


if __name__ == "__main__":
    main()
