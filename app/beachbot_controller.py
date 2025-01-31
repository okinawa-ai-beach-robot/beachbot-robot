# the future is now... (avoids printing pytoch warnings about deprecated functions to console)
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

from pathlib import Path
import platform, ctypes

if platform.system() == "Linux":
    # Workaround
    # Force libgomp to be loaded before other libraries consuming dynamic TLS (to avoid running out of STATIC_TLS)
    # Avoids error: "...libGLdispatch.so.0: cannot allocate memory in static TLS block"
    # Occurs on Jetson
    #preload_lib = Path("/lib/aarch64-linux-gnu/libGLdispatch.so.0")
    preload_lib = Path("/home/beachbot/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0")
    if preload_lib.is_file():
        ctypes.cdll.LoadLibrary(preload_lib.absolute().as_posix())

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
from beachbot.config import config
from beachbot.assets import get_asset_path
from beachbot.utils.system import MonitoredStdStreams

from beachbot.config import logger
from beachbot.robot.robotinterface import RobotInterface

from beachbot.ai.yolov5_torch_hub import Yolo5TorchHub
from beachbot.ai.blobdetectoropencv import BlobDetectorOpenCV
class BeachbotYolo5TorchHub(Yolo5TorchHub):
    def __init__(self, model_file=None, use_accel=True):
        if model_file is None:
            model_file = str(config.BEACHBOT_MODELS) + "/beachbot_yolov5s_beach-cleaning-object-detection__v8-yolotrain__yolov5pytorch_640_finetune/"
        super().__init__(model_file, use_accel)
model_list = [Yolo5TorchHub, BeachbotYolo5TorchHub, BlobDetectorOpenCV]
    

from beachbot.control.robotcontroller import BoxDef
from beachbot.control.controllerselector import ControllerSelector as MyController


from beachbot.utils.videowriteropencv import VideoWriterOpenCV

import time


from fastapi import Response

from nicegui import Client, app, core, run, ui
from nicegui import app, ui



from argparse import ArgumentParser




parser = ArgumentParser()
parser.add_argument("--sim", default=False, action="store_true", help="Execute in simulation instead of on real robot")
args = parser.parse_args()


target_obj="chair"




tab_names = ["Control", "Recordings"]


if args.sim:
    logger.info("Using simulation as --sim flag is set")
    from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
    robot = VrepRobotSimV1(scene="roarm_m1_locomotion_3finger.ttt")
else:
    logger.info("Using real robot as --sim flag is not set")
    from beachbot.robot.jetsonrobotv1 import JetsonRobotV1
    robot = JetsonRobotV1()

robot.start()
time.sleep(3)


live_update_timer = None







sleep_time = 0.1


#media = Path(VideoWriterOpenCV.get_base_path())
app.add_media_files("/my_videos", Path(VideoWriterOpenCV.get_base_path()))

# image placeholder in case no video device available:
black_1px = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAAAXNSR0IArs4c6QAAAA1JREFUGFdjYGBg+A8AAQQBAHAgZQsAAAAASUVORK5CYII="
placeholder = Response(content=base64.b64decode(black_1px.encode("ascii")), media_type="image/png")




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


@ui.refreshable
def ui_model_info(robot : RobotInterface):
    detector = robot.get_detector()
    if detector is not None:
        ui.label(f"Model: {detector.__class__.__name__}")
    else:
         ui.label("Model: None")



# with ui.dialog() as dialog, ui.card():
#     ui.label('Hello world!')
#     ui.button('Close', on_click=dialog.close)


# class UILoadDialog(ui.dialog):
#     def __init__(self):
#         super().__init__()
#         self.msg="none\n"
#         #self.updateui()

#     def updateui(self):
#         with ui.card():
#             ui.textarea(self.msg)
#             ui.button('Close', on_click=self.close)

#     def show_me(self):
#         self.open()


# diag = UILoadDialog()

async def toggle_detection(doit, ai_model=Yolo5TorchHub):
    global detector, video_image, diag
    video_image.content = ""
    print("Detection:", doit)
    if doit:
        # def f(s, file=sys.stdout):
        #     print("redirected", s, file=file)
        #     print("redirect end", file=file)
        # mys=MonitoredStream(sys.stdout, f)


        #diag = UILoadDialog()
        #diag.show_me()
        msg = ""

        def f_std(s, msg):
            print(s)
            msg += s
            #loadingdialog.refresh(msg)

        def f_err(s, msg):
            print(s)
            msg += s
            #loadingdialog.refresh(msg)

        with MonitoredStdStreams(lambda s: f_std(s, msg),lambda s: f_err(s, msg)): # 
            print("test")
            robot.set_detector(ai_model())

        
        # Set Inital confidence threshold for object detector
        try:
            robot.set_property("detector.conf_threshold", 0.3)
        except ValueError:
            logger.warning("Current detector does not support confidence thresholding")

    else:
        robot.set_detector(None)
    ui_model_info.refresh(robot)
    ui_config_panel.refresh()

def blobcfg():
    toggle_detection(True)




@ui.refreshable
def ui_config_panel(robot : RobotInterface) -> None:
    # TODO with ui.scroll_area().classes('w-full h-full border'):
    if robot is not None:
        for prop in robot.list_property_names():    
            value = robot.get_property(prop)
            value_bounds = robot.get_property_bounds(prop)
            with ui.row().classes("w-full justify-between no-wrap"):
                if type(value)==str:
                        ui.label("robot."+prop+":")
                        ui.input(label="robot."+prop, placeholder='enter string', value=value, on_change=lambda e, p=prop: robot.set_property(p, e.value))
                elif type(value)==float:
                        if value_bounds is not None and value_bounds[0] is not None and value_bounds[1] is not None:
                            ui.label("robot."+prop+":")
                            ui.slider(min=value_bounds[0], max=value_bounds[1], step=(value_bounds[1]-value_bounds[0])/255.0, value=value, on_change=lambda e, p=prop: robot.set_property(p, float(e.value))).props('label-always')
                        else:
                            ui.label("robot."+prop+":")
                            ui.number(label="robot."+prop, value=value, step=0.1, format='%.2f', on_change=lambda e, p=prop: robot.set_property(p, float(e.value)))
                elif type(value)==bool:
                        ui.checkbox("robot."+prop, value=value, on_change=lambda e, p=prop: robot.set_property(p, e.value))



def toggle_control(doit):
    if doit:
        robot.set_controller(MyController())
        ui_config_panel.refresh(robot)
    else:
        robot.set_controller(None)
        ui_config_panel.refresh(robot)

# def update_kp(new_kp):
#     if controller is not None:
#         controller.ctrl.kp=kp_slider.value

def toggle_recoding(doit):
    global video_is_recording
    if doit and not robot.is_recording():
        fname = robot.start_recording()
        print("Start recording into file", fname)
        video_is_recording = True
    elif not doit:
        fname = robot.stop_recording()
        if fname is not None:
            print("stopped recording into",fname)


def joystick_move(data):
    coordinates.set_text(f"{data.x:.3f}, {data.y:.3f}")
    # force_overwrite -> block motor commands from controller for e.g. 5 seconds
    robot.set_target_velocity(data.x * 100, data.y * 100, force_overwrite=5)


def joystick_end():
    coordinates.set_text("0, 0")
    # force_overwrite -> block motor commands from controller for e.g. 5 seconds
    robot.set_target_velocity(0, 0, force_overwrite=5) 


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
    global selector
    selector.clear()
    with selector:
        for dirpath, dirnames, filenames in walk(Path(VideoWriterOpenCV.get_base_path())):
            for fname in filenames:
                if fname.endswith(".mp4"):
                    ui.item(fname, on_click=lambda x=fname: change_media(x))


def convert(frame: np.ndarray) -> bytes:
    _frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    _, imencode_image = cv2.imencode(".jpg", _frame)
    return imencode_image.tobytes()



def add_imgbox(pleft=0, ptop=0, w=0, h=0, clsstr=None, color='#FF0000', conf_value=None, align="start"):
    svgstr=""
    if conf_value is not None:
        clsstr+=f"({conf_value:.2f})"
    # color = 'SkyBlue'
    if clsstr==target_obj:
        # Overwrite green color for followed object... 
        color="#00FF00"
    svgstr += f'<rect x="{pleft*100}%" y="{ptop*100}%" ry="15" height="{h*100}%" width="{w*100}%" fill="none" stroke="{color}" stroke-width="4" />'
    if clsstr is not None:
        if align=="start":
            svgstr += f'<text text-anchor="start" x="{pleft*100}%" y="{ptop*100}%" stroke="{color}" font-size="2em">{clsstr}</text>'
        else:
            svgstr += f'<text text-anchor="{align}" x="{(pleft+w)*100}%" y="{(ptop+h)*100}%" stroke="{color}" font-size="2em">{clsstr}</text>'
    return svgstr
    

def update_detection(robot:RobotInterface):
    frame, boxes = robot.get_buffered_camera_image()
    boxsvg = ""
    if boxes is not None:
        for b in boxes:
            boxsvg += add_imgbox(b.left, b.top, b.w, b.h, b.class_name, conf_value=b.confidence)
    return frame, boxsvg




@app.get("/video/frame")
# UI requests an update of the current detection result ... 
async def grab_video_frame() -> Response:
    frame, boxsvg = update_detection(robot)
    if frame is None:
        return placeholder
    

    # `convert` is a CPU-intensive function, so we run it in a separate process to avoid blocking the event loop and GIL.
    jpeg = await run.cpu_bound(convert, frame)
    video_image.content = boxsvg
    return Response(content=jpeg, media_type="image/jpeg")

with ui.tabs().classes("w-full") as tabs:
    # tabs.on('click', lambda s: reload_files())
    one = ui.tab(tab_names[0])
    two = ui.tab(tab_names[1])
tab_panel = ui.tab_panels(tabs, value=one).classes("w-full")
tab_panel.on_value_change(tab_select_event)
with tab_panel:
    with ui.tab_panel(one):
        with ui.row().classes("w-full"):
            toggle1 = ui.toggle(
                {1: "Video Stop", 2: "Record"}, value=1
            ).on_value_change(lambda v: toggle_recoding(v.value == 2))
            with ui.column():
                with ui.row().classes("w-full"):
                    with ui.dropdown_button('Select Model', auto_close=True):
                        ui.item("Detection Off", on_click=lambda: toggle_detection(False, None))
                        for model in model_list:
                            ui.item(str(model.__name__), on_click=lambda m=model: toggle_detection(True, m))
                    ui.space()
                    do_control = ui.switch('Robot Control', on_change=lambda x: toggle_control(x.value))
                
                with ui.row().classes("w-full justify-between no-wrap"):
                    ui_model_info(robot)
                    # ui.label("Model:")
                    # with ui.dropdown_button('Select Model', auto_close=True):
                    #     ui.item("Detection Off", on_click=lambda: toggle_detection(False, None))
                    #     for model in model_list:
                    #         ui.item(str(model), on_click=lambda m=model: toggle_detection(True, m))

            with ui.dropdown_button("System", auto_close=True):
                ui.item("Exit Server", on_click=app.shutdown)
                ui.item("Shut Down", on_click=sys_shutdown)
        with ui.splitter().classes("w-full h-full") as splitter:
            with splitter.before:
                ui.label("Robot Control Panel")
                with ui.tabs().classes("w-full") as tabs_ctrl:
                    one_ctrl = ui.tab("Locomotion")
                    two_ctrl = ui.tab("Arm")
                    three_ctrl = ui.tab("Arm Cart")
                    four_ctrl = ui.tab("Settings")
                tab_panel_ctrl = ui.tab_panels(tabs_ctrl, value=one_ctrl).classes("w-full")
                with tab_panel_ctrl.classes("w-full h-full"):
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
                    with ui.tab_panel(four_ctrl).classes('w-full h-full border'):
                        ui_config_panel(robot)

            with splitter.after:
                video_image = ui.interactive_image().classes("w-full h-full")
    with ui.tab_panel(two):
        with ui.dropdown_button("Select File...", auto_close=True) as selector:
            pass
        ui.label("Media Viewer:")
        ui.label(VideoWriterOpenCV.get_base_path())
        uivideo = ui.video("src")

reload_files()

# Start image view update timer:
live_update_timer = ui.timer(
    interval=0.5, callback=lambda: video_image.set_source(f"/video/frame?{time.time()}")
)




# disconnect clients (websocket) form server
async def disconnect() -> None:
    """Disconnect all clients from current running server."""
    for client_id in Client.instances:
        await core.sio.disconnect(client_id)


# Setup system handler for shutdown
def handle_sigint(signum, frame) -> None:
    app.shutdown()
# Catch Ctrl+C for shutdown
signal.signal(signal.SIGINT, handle_sigint)

# Cleanup routins on app shutdown
async def cleanup() -> None:
    # disconnect clients when the app is stopped with Ctrl+C
    await disconnect()

    print("Exit, cleaning up...")
    joystick_end()
    robot.cleanup()
app.on_shutdown(cleanup)


# Start app
ui.run(reload=False, port=8080, show=False, title="Beachbot", favicon=str(get_asset_path() / "beachbot_128x128.png"))
