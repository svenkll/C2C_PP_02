#!/usr/bin/env python
# Information: Module mit Basisklasse für Camera für Projekt RPiCar
# File name: basisklassen_cam.py
# Author: Robert Heise (FIDA)
# Date created: 11/10/2021
# Date last modified: 15/09/2023
# Python Version: 3.9
# Usage: U4I/FIDA "Autonomens Fahren" Sunfounder PiCar-S

import numpy as np
import cv2


class Camera(object):
    def __init__(
        self,
        devicenumber: int = 0,
        buffersize: int = 1,
        skip_frame: int = 0,
        height: int = None,
        width: int = None,
        flip: bool = True,
        colorspace: str = "bgr",
    ) -> None:
        """Initializes Camera

        Args:
            devicenumber (int, optional): Identifies camera to be used.
                0 for default camera. Defaults to 0.
            buffersize (int, optional): _description_. Defaults to 1.
            skip_frame (int, optional): _description_. Defaults to 0.
            height (int, optional): height of images. Defaults to None -> height defaults to camera.
            width (int, optional): height. Defaults to None -> width defaults to camera
            flip (bool, optional): mirrow image on the horizonal axis. Defaults to True.
            colorspace (str, optional): 'bgr', 'rgb', 'gray' Defaults to 'bgr'.

        Note:
            Note: Some camera do not support a free choice of height and width.
            When trying to set width or height, they default to supported formats.
            This does not concern the camera used on the RPi!
        """
        self.__devicenumber = devicenumber
        self.__VideoCapture = cv2.VideoCapture(self.__devicenumber)
        if width:
            self.__VideoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.__VideoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        width_active = int(self.__VideoCapture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height_active = int(self.__VideoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.__imgsize = (width_active, height_active)
        self.__VideoCapture.set(cv2.CAP_PROP_BUFFERSIZE, buffersize)
        self.__flip = flip
        self.__colorspace = colorspace or "bgr"
        self.__skip_frame = skip_frame

    def get_size(self) -> tuple:
        """Return size of frame returned by get_frame.

        Returns:
            tuple: (height, width, colors)
        """
        return self.__imgsize

    def get_frame(self) -> np.array:
        """Returns frame with tranformations applied

        Raises:
            Exception: Exception, if camera is not accessible

        Returns:
            np.array: frame representing the image
        """
        if not self.__VideoCapture.isOpened():
            raise Exception("Could not open video device")
        else:
            if self.__skip_frame:
                for i in range(int(self.__skip_frame)):
                    _, frame = self.__VideoCapture.read()
            _, frame = self.__VideoCapture.read()
            if self.__flip:
                frame = cv2.flip(frame, -1)
            if self.__colorspace == "rgb":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            elif self.__colorspace == "gray":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                pass
        return frame

    def check(self) -> bool:
        """Test for accessibility of the camera

        Returns:
            bool: True if videocapture exists and is open -> accesses camera
        """
        flag = self.__VideoCapture is not None and self.__VideoCapture.isOpened()
        return flag

    def release(self) -> None:
        """Releases camera and allows other processes to access it_"""
        self.__VideoCapture.release()


if __name__ == "__main__":

    cam = Camera()
    while True:
        img = cam.get_frame()
        cv2.imshow("image", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cam.release()
    print(" - camera released")
    cv2.destroyAllWindows()
