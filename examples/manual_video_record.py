import beachbot
import time

from beachbot.utils.imageviewermatplotlib import ImageViewerMatplotlib
from beachbot.sensors.jetsoncsicameraopencv import JetsonCsiCameraOpenCV as cameradevice
from beachbot.sensors.usbcameraopencv import UsbCameraOpenCV

from beachbot.utils.videowriteropencv import VideoWriterOpenCV as videodevice



# A reference to a image viewer can be used to display images:
viewer = ImageViewerMatplotlib


# There are two possible backend to read images from the camera devices
#cam1 = beachbot.sensors.UsbCameraOpenCV(width=640, height=480, fps=30, dev_id=0)
cam1 = cameradevice()
#cam1 = beachbot.sensors.JetsonGstCameraNative()

if isinstance(cam1, UsbCameraOpenCV):
    # just for fun: print list of video devices and current resolution to console:
    cam1.list_cameras()


# retrieve information on video stream:
capture_width, capture_height = cam1.get_size()

# Create video file writer, for now only one backend is implemented (opencv):
videowriter = videodevice("filename_tmp.mp4", fps=10, capture_width=capture_width, capture_height=capture_height )


print("My resolution is:", cam1.get_size())

time.sleep(1)

# read first frame of camera:
img1 = cam1.read()

# Instantiate the image viewer class and display the first frame
w1 = viewer("Title")
w1.show(img1)


# The following loop reads frames from the camera and writes them to the video file:
try:
    for i in range(200):
        # read frame:
        img1 = cam1.read()

        # display camera image:
        w1.show(img1)

        # Write into file:
        videowriter.add_frame(img1)

        # Wait for a certain amount of time:
        time.sleep(0.1)
except KeyboardInterrupt as ex:
    pass

# Do not forget to close preview, video devices and video writer objects, 
# - Camera access could be blocked if not closed correctly
# - The frames may only be written to the file during closing as they can be buffered im memory
w1.close()
videowriter.close()
cam1.stop()

