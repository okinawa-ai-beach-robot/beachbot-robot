import os, threading
import numpy as np
import cv2

from ..utils.vrepsimulation import vrep
from beachbot.config import logger


class VrepCameraSim():
    def __init__(self, vrep_sim, cam_name, perspective_angle_overwrite=None, rotation_offset=None) -> None:
        """
        Args:
        perspective_angle_overwrite: perspective angle (fov) of camera, default is 100 (in degree)
        rotation_offset: Additional rotation (up/down shift of horizon) of the camera, -10 (degree) moves camera vier upwards
        """
        # Init superclass thread
        super().__init__()
        # do not block on exit:
        self.vrep_sim = vrep_sim
        self._cam_name=cam_name

        self._init_sim(cam_name, perspective_angle_overwrite, rotation_offset)

        img, [resX, resY] = self._read_visionsensor()
        self._width=resX
        self._height=resY

        self._stopped=True

        img = np.frombuffer(img, dtype=np.uint8).reshape(self._height, self._width, 3)
        self._frame = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)


    @vrep
    def _init_sim(self, cam_name, perspective_angle_overwrite=None, rotation_offset=None):
        self._cam_id = self.vrep_sim.getObject("/"+cam_name)
        if perspective_angle_overwrite is not None:
            logger.info(f"Overwrite camera parameter fov (perspective_angle_overwrite) with {perspective_angle_overwrite}")
            self.vrep_sim.setObjectFloatParam(self._cam_id, self.vrep_sim.visionfloatparam_perspective_angle, np.pi*(perspective_angle_overwrite/180.0))
        if rotation_offset is not None:
            logger.info(f"Setting camera rotation offset (x-axis) to {rotation_offset}")
            self.vrep_sim.setObjectOrientation(self._cam_id, [np.pi*(rotation_offset/180.0),0,0], self._cam_id)



    @vrep
    def _read_visionsensor(self):
        try:
            img, [resX, resY] = self.vrep_sim.getVisionSensorImg(self._cam_id)
            return img, [resX, resY]
        except:
            return None, (0,0)



    @staticmethod
    def list_cameras():
        print("STUB list_cameras(): Simulation")


    def start(self):
        self._stopped = True


    def read(self):
        img, [resX, resY] = self._read_visionsensor()
        if img is None:
            return None
        self._width=resX
        self._height=resY
        img = np.frombuffer(img, dtype=np.uint8).reshape(self._height, self._width, 3)
        self._frame = cv2.flip(img, 0) #cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return self._frame

    def stop(self):
        self._stopped = True

    def is_running(self):
        return not self._stopped

    def get_size(self):
        if self._frame.shape is not None:
            return (self._frame.shape[1], self._frame.shape[0])
        return (self._width, self._height)
