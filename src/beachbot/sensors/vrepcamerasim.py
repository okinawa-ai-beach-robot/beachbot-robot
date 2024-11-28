import os, threading
import numpy as np
import cv2


class VrepCameraSim():
    def __init__(self, vrep_sim, cam_name) -> None:
        # Init superclass thread
        super().__init__()
        # do not block on exit:
        self.vrep_sim = vrep_sim
        self._cam_name=cam_name
        self._cam_id = self.vrep_sim.getObject("/"+cam_name)

        img, [resX, resY] = self.vrep_sim.getVisionSensorImg(self._cam_id)
        self._width=resX
        self._height=resY

        self._stopped=True

        img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
        self._frame = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)



    @staticmethod
    def list_cameras():
        print("STUB list_cameras(): Simulation")


    def start(self):
        self._stopped = True


    def read(self):
        print("Read cam:", self._cam_id, self._cam_name)
        img, [resX, resY] = self.vrep_sim.getVisionSensorImg(self._cam_id)
        self._width=resX
        self._height=resY
        img = np.frombuffer(img, dtype=np.uint8).reshape(self._height, self._width, 3)
        self._frame = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        return self._frame

    def stop(self):
        self._stopped = True

    def is_running(self):
        return not self._stopped

    def get_size(self):
        if self._frame.shape is not None:
            return (self._frame.shape[1], self._frame.shape[0])
        return (self._width, self._height)
