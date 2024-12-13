import os, threading
import time
import cv2

from beachbot.config import config, logger

class UsbCameraOpenCV(threading.Thread):
    def __init__(self, width=640, height=480, fps=25, dev_id=1, autostart=True) -> None:
        # Init superclass thread
        super().__init__()
        # do not block on exit:
        self.daemon = True
        self._stopped = True
        self._frame = None

        self._lock = threading.Lock()

        self._width=width
        self._height=height
        self._fps=fps
        self._dev_id=dev_id

        if autostart:
            self.start()



    @staticmethod
    def list_cameras():
        print(os.popen("v4l2-ctl --list-devices").read())
        print(os.popen("v4l2-ctl -d /dev/video1 --list-formats-ext").read())

    def start(self):
        if self._stopped:
            self._cap = cv2.VideoCapture(self._dev_id)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)

        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.ret_val, bgr_frame = self._cap.read()
            self._frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            self._stopped = False
            super().start()
        else:
            logger.error(f"Could not open camera, device available, resolution and framerate supported? check with v4l2-ctl -d /dev/video{self._dev_id} --list-formats-ext")
            raise Exception(f"Could not open /dev/video{self._dev_id}")


    @staticmethod
    def list_cameras():
        print(os.popen("v4l2-ctl --list-devices").read())
        print(os.popen("for d in /dev/video* ; do echo $d ; v4l2-ctl --device=$d -D --list-formats-ext  ; echo '===============' ; done").read())

    def run(self):
        while not self._stopped:
            self._ret, bgr_frame = self._cap.read()
            with self._lock:
                self._frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        self._cap.release()

    def read(self):
        with self._lock:
            img_cpy = self._frame.copy()
        return img_cpy

    def stop(self):
        self._stopped = True
        counter=0
        while(self._cap.isOpened() and counter<10):
            time.sleep(0.1)
        if counter==10:
            logger.error("Could not release camera!s")


    def is_running(self):
        return not self._stopped

    def get_size(self):
        if self._frame.shape is not None:
            return (self._frame.shape[1], self._frame.shape[0])
        return (self._width, self._height)
        # This does not reflect actual size: return (int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
