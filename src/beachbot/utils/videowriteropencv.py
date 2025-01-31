import cv2
import datetime, time
import threading
import os


from beachbot.config import config


class VideoWriterOpenCV(threading.Thread):
    def __init__(
        self, filename=None, fps=10, capture_width=1920, capture_height=1080
    ) -> None:
        # Init superclass thread
        super().__init__()
        # do not block on exit:
        self.daemon = True

        if filename is None:
            basepath = VideoWriterOpenCV.get_base_path()
            filename = basepath + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        
        if "." not in filename.rsplit(os.path.sep, 1)[-1]:
            filename += ".mp4"

        fourcc = cv2.VideoWriter_fourcc(*"h264")
        # or
        # fourcc = cv2.VideoWriter_fourcc(*'x264')
        # fourcc = cv2.VideoWriter_fourcc(*'avc1')
        # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        # cfourcc = v2.VideoWriter_fourcc('M','J','P','G')
        ##sudo apt-get install ffmpeg x264 libx264-dev

        # adjust qiality with pipeline spec:
        # writer.open("appsrc ! videoconvert  !  avenc_mpeg4 bitrate=100000 ! mp4mux ! filesink location=video.mp4",CAP_GSTREAMER, 0, (double) 10, cv::Size(640, 480), true);
        self.vidfile = cv2.VideoWriter(
            filename, fourcc, fps, (capture_width, capture_height)
        )
        self.filename = filename
        self._fps = fps
        self._videosource = None
        self._stopped = True

    @staticmethod
    def get_base_path():
        basepath = str(config.BEACHBOT_RECORDINGS) + os.path.sep
        os.makedirs(basepath, exist_ok=True)
        return basepath

    def add_frame(self, frame):
        _frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.vidfile.write(_frame)

    def start_recording(self, videosource):
        self._stopped = False
        self._videosource = videosource
        super().start()

    def run(self):
        while not self._stopped:
            t_start = time.time()
            img = self._videosource.read()
            self.add_frame(img)
            t_end = time.time()
            t_wait = (1.0 / self._fps) - (t_end - t_start)
            if t_wait > 0:
                time.sleep(t_wait)

    def close(self):
        if not self._stopped:
            self._stopped = True
            time.sleep(1.0 / self._fps)
            
        self.vidfile.release()

