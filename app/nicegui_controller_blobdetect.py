import requests

from pathlib import Path
from os import walk

import base64
import signal
import time

import cv2
import numpy as np


import sys
import signal
#from beachbot.manipulators import Motor, DifferentialDrive
#import beachbot.sensors
import beachbot
from beachbot.config import logger
from beachbot.robot.robotinterface import RobotInterface
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from beachbot.robot.jetsonrobotv1 import JetsonRobotV1
from beachbot.ai.blobdetectoropencv import BlobDetectorOpenCV
from beachbot.control.robotcontroller import BoxDef
from beachbot.control.approachdebris import ApproachDebris as ApproachDebrisController


from beachbot.utils.videowriteropencv import VideoWriterOpenCV

import time


from fastapi import Response

from nicegui import Client, app, core, run, ui
from nicegui import app, ui



from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--sim", default=False, action="store_true", help="Execute in simulation instead of on real robot")
args = parser.parse_args()




tab_names = ["Control", "Recordings"]


if args.sim:
    robot = VrepRobotSimV1(scene="roarm_m1_locomotion_3finger.ttt")
else:
    logger.info("Using real robot as --sim flag is not set")
    robot = JetsonRobotV1()
    
cam1 = robot.cameradevices[RobotInterface.CAMERATYPE.FRONT]
# retrieve information on video stream:
capture_width, capture_height = cam1.get_size()
# Create video file writer, for now only one backend is implemented (opencv):
videowriter = None  # beachbot.utils.VideoWriterOpenCV(None, fps=10, capture_width=capture_width, capture_height=capture_height )


video_is_recording = False






sleep_time = 0.1


media = Path(VideoWriterOpenCV.get_base_path())
app.add_media_files("/my_videos", media)

# image placeholder in case no video device available:
black_1px = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAAAXNSR0IArs4c6QAAAA1JREFUGFdjYGBg+A8AAQQBAHAgZQsAAAAASUVORK5CYII="
placeholder = Response(
    content=base64.b64decode(black_1px.encode("ascii")), media_type="image/png"
)




def arm_action_cartesian():
    x = cart_x_slider.value
    y = cart_y_slider.value
    z = cart_z_slider.value
    r = cart_r_slider.value
    robot.arm.set_cart_pos([x,y,z], r)
    
    

def arm_action_home():
    print("Arm go home")
    robot.arm.go_home()


def arm_action_zero():
    print("Arm go zero")
    robot.arm.go_zero()

def arm_action_calib():
    print("Arm go zero")
    robot.arm.go_calib()

async def arm_action_test():
    print("Arm go test")
    await run.io_bound(robot.arm.test_movement)

detector = None
def toggle_detection(doit):
    global detector, video_image
    video_image.content = ""
    print("Detection:", doit)
    if doit:
        detector = BlobDetectorOpenCV(minArea=blob_mi.value, maxArea=blob_ma.value)
        detector.lower_blue=np.array([int(blob_hmi.value), int(blob_smi.value), int(blob_vmi.value)])
        detector.upper_blue=np.array([int(blob_hma.value), int(blob_sma.value), int(blob_vma.value)])
        print(blob_mi.value, blob_ma.value)
        print(detector.lower_blue)
        print(detector.upper_blue)
    else:
        detector=None

def blobcfg():
    toggle_detection(True)


controller = None
def toggle_control(doit):
    global controller
    print("Control:", doit)
    if doit:
        controller = ApproachDebrisController()
        controller.ctrl.kp=kp_slider.value
    else:
        controller=None

def update_kp(new_kp):
    if controller is not None:
        controller.ctrl.kp=kp_slider.value

def toggle_recoding(doit):
    global video_is_recording, videowriter
    if doit and not video_is_recording:
        videowriter = VideoWriterOpenCV(
            None, fps=10, capture_width=capture_width, capture_height=capture_height
        )
        # initiate threaded recording into video file
        videowriter.start_recording(cam1)
        print("Start recording")
        video_is_recording = True
    elif not doit:
        if videowriter is not None:
            videowriter.close()
            print("Stop recording")
        video_is_recording = False


def joystick_move(data):
    coordinates.set_text(f"{data.x:.3f}, {data.y:.3f}")
    robot.set_target_velocity(data.x * 100, data.y * 100)


def joystick_end():
    coordinates.set_text("0, 0")
    robot.set_target_velocity(0, 0)


def sys_shutdown():
    print("Bye bye ...")
    beachbot.utils.shutdown()


def change_media(file):
    print("load video:", "/my_videos/" + file)
    uivideo.set_source("/my_videos/" + file)


def tab_select_event():
    global live_update_timer, tab_names, video_image
    try:
        if tab_panel.value == tab_names[0]:
            if live_update_timer is None:
                live_update_timer = ui.timer(
                    interval=0.5,
                    callback=lambda: video_image.set_source(
                        f"/video/frame?{time.time()}"
                    ),
                )
        else:
            if live_update_timer is not None:
                live_update_timer.cancel()
                live_update_timer = None
        if tab_panel.value == tab_names[1]:
            print("reload files...")
            reload_files()
    except Exception as ex:
        print(ex)


def reload_files():
    global selector, media
    selector.clear()
    with selector:
        for dirpath, dirnames, filenames in walk(media):
            for fname in filenames:
                if fname.endswith(".mp4"):
                    ui.item(fname, on_click=lambda x=fname: change_media(x))


def convert(frame: np.ndarray) -> bytes:
    _frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    _, imencode_image = cv2.imencode(".jpg", _frame)
    return imencode_image.tobytes()



def add_imgbox(pleft=0, ptop=0, w=0, h=0, clsstr=None, color='#FF0000', align="start"):
    # color = 'SkyBlue'
    video_image.content += f'<rect x="{pleft*100}%" y="{ptop*100}%" ry="15" height="{h*100}%" width="{w*100}%" fill="none" stroke="{color}" stroke-width="4" />'
    if clsstr is not None:
        if align=="start":
            video_image.content += f'<text text-anchor="start" x="{pleft*100}%" y="{ptop*100}%" stroke="{color}" font-size="2em">{clsstr}</text>'
        else:
            video_image.content += f'<text text-anchor="{align}" x="{(pleft+w)*100}%" y="{(ptop+h)*100}%" stroke="{color}" font-size="2em">{clsstr}</text>'
    

def detection(frame, detector, controller=None):
    class_ids, confidences, boxes = detector.apply_model(frame)
    video_image.content = ""
    boxlist = []
    for classid, confidence, box in zip(class_ids, confidences, boxes):
        if confidence >= 0.01:
            add_imgbox(*box, detector.list_classes[classid])
            boxlist.append(BoxDef(left=box[0], top=box[1], w=box[2], h=box[3], class_name=detector.list_classes[classid]))
    if controller is not None:
        controller.update(robot, boxlist)


@app.get("/video/frame")
# Thanks to FastAPI's `app.get`` it is easy to create a web route which always provides the latest image from OpenCV.
async def grab_video_frame() -> Response:
    # The `video_capture.read` call is a blocking function.
    # So we run it in a separate thread (default executor) to avoid blocking the event loop.
    frame = await run.io_bound(cam1.read)
    if frame is None:
        return placeholder
    
    if detector is not None:
        await run.io_bound(detection, frame, detector, controller)

    # `convert` is a CPU-intensive function, so we run it in a separate process to avoid blocking the event loop and GIL.
    jpeg = await run.cpu_bound(convert, frame)
    return Response(content=jpeg, media_type="image/jpeg")

with ui.tabs().classes("w-full") as tabs:
    # tabs.on('click', lambda s: reload_files())
    one = ui.tab(tab_names[0])
    two = ui.tab(tab_names[1])
tab_panel = ui.tab_panels(tabs, value=two).classes("w-full")
tab_panel.on_value_change(tab_select_event)
with tab_panel:
    with ui.tab_panel(one):
        with ui.row().classes("w-full"):
            toggle1 = ui.toggle(
                {1: "Video Stop", 2: "Record"}, value=1
            ).on_value_change(lambda v: toggle_recoding(v.value == 2))
            with ui.column():
                with ui.row().classes("w-full"):
                    do_detect = ui.switch('Blob Detection', on_change=lambda x: toggle_detection(x.value))
                    ui.space()
                    do_control = ui.switch('Robot Control', on_change=lambda x: toggle_control(x.value))
                
                with ui.row().classes("w-full justify-between no-wrap"):
                    ui.label("kp:")
                    kp_slider = ui.slider(min=0.1, max=100, step=0.1, value=0.1, on_change=lambda x: update_kp(x.value)).props('label')

            with ui.dropdown_button("System", auto_close=True):
                ui.item("Exit Server", on_click=app.shutdown)
                ui.item("Shut Down", on_click=sys_shutdown)
        with ui.splitter().classes("w-full") as splitter:
            with splitter.before:
                ui.label("Robot Control Panel")
                with ui.tabs().classes("w-full") as tabs_ctrl:
                    one_ctrl = ui.tab("Locomotion")
                    two_ctrl = ui.tab("Arm")
                    three_ctrl = ui.tab("Arm Cart")
                    four_ctrl = ui.tab("Settings")
                tab_panel_ctrl = ui.tab_panels(tabs_ctrl, value=one_ctrl).classes("w-full")
                with tab_panel_ctrl:
                    with ui.tab_panel(one_ctrl):
                        ui.add_head_html(
                                    """
                                    <style>
                                        .custom-joystick[data-joystick]{
                                            width: 90%;
                                            height: auto;
                                            max-height: 60vh;
                                            aspect-ratio: 1 / 1;
                                        }
                                    </style>
                                    """
                        )
                        ui.joystick(
                            color="blue",
                            size=350,
                            on_move=lambda e: joystick_move(e),
                            on_end=lambda _: joystick_end(),
                        ).classes("custom-joystick")
                        coordinates = ui.label("0, 0")
                    with ui.tab_panel(two_ctrl):
                        ui.label("test")
                        ui.button("Go Home", on_click=lambda x: arm_action_home())
                        ui.button("Go Calib", on_click=lambda x: arm_action_calib())
                        ui.button("Go Zero", on_click=lambda x: arm_action_zero())
                        ui.button("Go Test", on_click=lambda x: arm_action_test())
                    with ui.tab_panel(three_ctrl):
                        ui.button("Activate", on_click=lambda x: arm_action_cartesian())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("x:")
                            cart_x_slider = ui.slider(min=-200, max=200, step=1, value=0.0, on_change=lambda x: arm_action_cartesian()).props('label')
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("y:")
                            cart_y_slider = ui.slider(min=-200, max=200, step=1, value=0.0, on_change=lambda x: arm_action_cartesian()).props('label')
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("z:")
                            cart_z_slider = ui.slider(min=-200, max=100, step=1, value=0.0, on_change=lambda x: arm_action_cartesian()).props('label')
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("r:")
                            cart_r_slider = ui.slider(min=-45, max=45, step=1.0, value=0.0, on_change=lambda x: arm_action_cartesian()).props('label')
                    with ui.tab_panel(four_ctrl):
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("H min:")
                            blob_hmi = ui.slider(min=0, max=255, step=1, value=60, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("H max:")
                            blob_hma = ui.slider(min=0, max=255, step=1, value=180, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("S min:")
                            blob_smi = ui.slider(min=0, max=255, step=1, value=60, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("S max:")
                            blob_sma = ui.slider(min=0, max=255, step=1, value=255, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("V min:")
                            blob_vmi = ui.slider(min=0, max=255, step=1, value=60, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("V max:")
                            blob_vma = ui.slider(min=0, max=255, step=1, value=255, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("min size:")
                            blob_mi = ui.slider(min=0, max=300000, step=100, value=1000, on_change=lambda x: blobcfg())
                        with ui.row().classes("w-full justify-between no-wrap"):
                            ui.label("max size:")
                            blob_ma = ui.slider(min=0, max=300000, step=100, value=200000, on_change=lambda x: blobcfg())
            with splitter.after:
                video_image = ui.interactive_image().classes("w-full h-full")
    with ui.tab_panel(two):
        with ui.dropdown_button("Select File...", auto_close=True) as selector:
            pass
        ui.label("Media Viewer:")
        ui.label(VideoWriterOpenCV.get_base_path())
        uivideo = ui.video("src")

reload_files()

# TODO add, only timing when view is on preview!!!
live_update_timer = ui.timer(
    interval=0.5, callback=lambda: video_image.set_source(f"/video/frame?{time.time()}")
)


# media.mkdir(exist_ok=True)
# r = requests.get('https://cdn.coverr.co/videos/coverr-cloudy-sky-2765/1080p.mp4')
# (media  / 'clouds.mp4').write_bytes(r.content)


async def disconnect() -> None:
    """Disconnect all clients from current running server."""
    for client_id in Client.instances:
        await core.sio.disconnect(client_id)


def handle_sigint(signum, frame) -> None:
    # `disconnect` is async, so it must be called from the event loop; we use `ui.timer` to do so.
    ui.timer(0.1, disconnect, once=True)
    # Delay the default handler to allow the disconnect to complete.
    ui.timer(1, lambda: signal.default_int_handler(signum, frame), once=True)


async def cleanup() -> None:
    # This prevents ugly stack traces when auto-reloading on code change,
    # because otherwise disconnected clients try to reconnect to the newly started server.
    await disconnect()

    print("Exit, cleaning up...")
    joystick_end()
    if videowriter is not None:
        videowriter.close()
    robot.cleanup()
    # sys.exit(0)


app.on_shutdown(cleanup)
# We also need to disconnect clients when the app is stopped with Ctrl+C,
# because otherwise they will keep requesting images which lead to unfinished subprocesses blocking the shutdown.
signal.signal(signal.SIGINT, handle_sigint)


ui.run(reload=False, port=8080, show=False)
