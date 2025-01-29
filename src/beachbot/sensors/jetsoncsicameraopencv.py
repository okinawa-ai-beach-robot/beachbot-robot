import os, threading
import cv2



""" 
gstreamer_pipeline_builder returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""
def gstreamer_pipeline_builder(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=-1,
    display_height=-1,
    framerate=15,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width if display_width > 0 else capture_width,
            display_height if display_height > 0 else capture_height,
        )
    )

class JetsonCsiCameraOpenCV(threading.Thread):
    def __init__(self, width=1280, height=720, fps=15, dev_id=0, autostart=True) -> None:
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
        print(os.popen("v4l2-ctl --list-formats-ext").read())

    def start(self):
        if "DISPLAY" not in os.environ:
            os.environ['DISPLAY'] = ':0.0'
        self._cap = cv2.VideoCapture(
            gstreamer_pipeline_builder(
                sensor_id=self._dev_id,
                capture_width=self._width,
                capture_height=self._height,
                framerate=self._fps,
                flip_method=2,
            ),
            cv2.CAP_GSTREAMER,
        )

        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.ret_val, self._frame = self._cap.read()
            #self._frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            self._stopped = False
            super().start()

    def run(self):
        while not self._stopped:
            # TODO check: self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.ret_val, bgr_frame = self._cap.read()
            with self._lock:
                self._frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        self._cap.release()

    def read(self):
        with self._lock:
            img_cpy = self._frame.copy()
        return img_cpy

    def stop(self):
        if self.is_running():
            self._stopped = True
            self._cap.release()

    def is_running(self):
        return not self._stopped

    def get_size(self):
        return (
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
