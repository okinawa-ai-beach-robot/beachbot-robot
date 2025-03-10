# the future is now... (avoids printing pytoch warnings about deprecated functions to console)
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)




from datetime import datetime
import io, os


import beachbot.manipulators
import beachbot.manipulators.drive
import beachbot.robot



from pathlib import Path
import platform, ctypes



if platform.system() == "Linux":
    # Workaround
    # Force libgomp to be loaded before other libraries consuming dynamic TLS (to avoid running out of STATIC_TLS)
    # Avoids error: "...libGLdispatch.so.0: cannot allocate memory in static TLS block"
    # Occurs on Jetson
    preload_lib = Path("/lib/aarch64-linux-gnu/libGLdispatch.so.0")
    if preload_lib.is_file():
        ctypes.cdll.LoadLibrary(preload_lib.absolute().as_posix())
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
from beachbot.config import config, logger
import logging
from beachbot.assets import get_asset_path


from beachbot.robot.robotinterface import RobotInterface
from beachbot.manipulators.drive import DifferentialDrive

from beachbot.ai.yolov5_torch_hub import Yolo5TorchHub, BeachbotYolo5TorchHub
from beachbot.ai.blobdetectoropencv import BlobDetectorOpenCV
# class BeachbotYolo5TorchHub(Yolo5TorchHub):
#     def __init__(self, model_file=None, use_accel=True):
#         if model_file is None:
#             model_file = str(config.BEACHBOT_MODELS) + "/beachbot_yolov5s_beach-cleaning-object-detection__v8-yolotrain__yolov5pytorch_640_finetune/"
#         super().__init__(model_file, use_accel)
model_list = [Yolo5TorchHub, BeachbotYolo5TorchHub, BlobDetectorOpenCV]
    

from beachbot.control.controllerselector import ControllerSelector
from beachbot.control.approachdebris import ApproachDebris
from beachbot.control.pickupcontroller import PickupController
from beachbot.control.adaptivepicker import AdaptivePickupController


controller_list = [ControllerSelector, ApproachDebris, PickupController, AdaptivePickupController]


from beachbot.utils.videowriteropencv import VideoWriterOpenCV

import time


from fastapi import Response

from nicegui import Client, app, core, run, ui
from nicegui import app, ui



from argparse import ArgumentParser




parser = ArgumentParser()
parser.add_argument("--sim", default=False, action="store_true", help="Execute in simulation instead of on real robot")
parser.add_argument("--cfgload", default=False, action="store_true", help="Reload stored configuration on startup")
args = parser.parse_args()


print("Beachbot startup\n config:\n", vars(config))

target_obj="none"


robot_config_filename = str(config.BEACHBOT_CONFIG / "robo_config.json")


tab_names = ["Control", "Recordings"]

if args.sim:
    logger.info("Using simulation as --sim flag is set")
    from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
    robot = VrepRobotSimV1(scene="beachbot_roarm_3finger.ttt")
else:
    logger.info("Using real robot as --sim flag is not set")
    from beachbot.robot.jetsonrobotv1 import JetsonRobotV1
    robot = JetsonRobotV1()

robot.start()

logger.info("Wait (3s) for initalization...")
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
    
    
def arm_calib_init():
    robot.arm.calib_init()
def arm_calib_free():
    robot.arm.calib_free()
def arm_calib_lock():
    robot.arm.calib_lock()
def arm_calib_save():
    robot.arm.calib_save()
        



def arm_action_home():
    print("Arm go home")
    robot.arm.go_home()


def arm_action_zero():
    print("Arm go zero")
    robot.arm.go_zero()

def arm_action_calib():
    print("Arm go claib")
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
    ui.space()
    controller = robot.get_controller()
    if controller is not None:
        ui.label(f"Controller: {controller.__class__.__name__}")
    else:
         ui.label("Controller: None")



async def toggle_detection(ai_model=Yolo5TorchHub):
    global video_image
    video_image.content = ""
    print("Detection:", ai_model)
    if ai_model is not None:
        robot.set_detector(ai_model())
    else:
        robot.set_detector(None)
    ui_model_info.refresh(robot)
    ui_config_panel.refresh()



async def toggle_controller(robot_controller=ControllerSelector):
    if robot_controller is not None:
        robot.set_controller(robot_controller())
    else:
        robot.set_controller(None)
    ui_model_info.refresh(robot)
    ui_config_panel.refresh()
    print("Controller:", robot_controller)



def update_target_obj(robot : RobotInterface) -> None:
    global target_obj
    propbe_props = ["controller.approach.targetfilter", "controller.targetfilter"]

    for prop in propbe_props:
        try:
            val = robot.get_property(prop)
            if val is not None:
                target_obj = str(val).split(",")
                return
        except ValueError:
            # Controller not loaded, do not update
            pass



def update_string_prop(robot : RobotInterface, name, val):
    global target_obj
    robot.set_property(name, val)
    if name == "controller.approach.targetfilter" or name == "controller.targetfilter":
        target_obj = str(val).split(",")


@ui.refreshable
def ui_config_panel(robot : RobotInterface) -> None:
    # TODO with ui.scroll_area().classes('w-full h-full border'):
    if robot is not None:
        prop_classes={}
        for prop in robot.list_property_names():    
            # update global target object list for box drawing:
            update_target_obj(robot)


            value = robot.get_property(prop)
            value_bounds = robot.get_property_bounds(prop)
            if "." in prop:
                value_subclass = prop.rsplit(".", 1)[0]
            else:
                value_subclass = None
            

            if value_subclass:
                if value_subclass in prop_classes:
                    targetelement_parent = prop_classes[value_subclass]
                else:
                    targetelement_parent =  ui.expansion("robot."+value_subclass+':', icon='tune').classes("w-full justify-between no-wrap")
                    prop_classes[value_subclass] = targetelement_parent

                with targetelement_parent:
                    targetelement = ui.row().classes("w-full justify-between no-wrap")
            else:
                targetelement = ui.row().classes("w-full justify-between no-wrap")

            with targetelement:
                tooltipelement = None
                if type(value)==str:
                        tooltipelement= ui.label("robot."+prop+":")
                        ui.input(label="robot."+prop, placeholder='enter string', value=value, on_change=lambda e, p=prop: update_string_prop(robot, p, e.value))
                elif type(value)==float or type(value)==int:
                        if value_bounds is not None and value_bounds[0] is not None and value_bounds[1] is not None:
                            tooltipelement = ui.label("robot."+prop+":")
                            ui.slider(min=value_bounds[0], max=value_bounds[1], step=(value_bounds[1]-value_bounds[0])/255.0, value=value, on_change=lambda e, p=prop: robot.set_property(p, float(e.value))).props('label-always')
                        else:
                            tooltipelement = ui.label("robot."+prop+":")
                            ui.number(label="robot."+prop, value=value, step=0.1, format='%.2f', on_change=lambda e, p=prop: robot.set_property(p, float(e.value)))
                elif type(value)==bool:
                        tooltipelement = ui.label("robot."+prop+":")
                        ui.checkbox(value=value, on_change=lambda e, p=prop: robot.set_property(p, e.value))

                tooltipstr = robot.get_property_description(prop)
                if tooltipelement is not None and tooltipstr is not None:
                    tooltipelement.tooltip(tooltipstr)




# def toggle_control(doit):
#     if doit:
#         robot.set_controller(MyController())
#         ui_config_panel.refresh(robot)
#     else:
#         robot.set_controller(None)
#         ui_config_panel.refresh(robot)

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
    print("TODO hardcoded password!!")
    os.system("echo beachbot | sudo -S poweroff")


def change_media(file):
    print("load video:", "/my_videos/" + file)
    uivideo.set_source("/my_videos/" + file)





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
    global target_obj
    svgstr=""

    if clsstr in target_obj:
        # Overwrite green color for followed object... 
        color="#00FF00"

    if conf_value is not None:
        clsstr+=f"({conf_value:.2f})"
    # color = 'SkyBlue'

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
    
    # `convert` is a CPU-intensive function, so we run it in a separate process to avoid blocking the event loop and GIL. TODO cpu_bound blocks infinitely
    jpeg = await run.io_bound(convert, frame)
    video_image.content = boxsvg
    return Response(content=jpeg, media_type="image/jpeg")

with ui.tabs().classes("w-full") as tabs:
    # tabs.on('click', lambda s: reload_files())
    one = ui.tab(tab_names[0])
    two = ui.tab(tab_names[1])
tab_panel = ui.tab_panels(tabs, value=one).classes("w-full")

with tab_panel:
    with ui.tab_panel(one):
        with ui.row().classes("w-full"):
            toggle1 = ui.toggle(
                {1: "Video Stop", 2: "Record"}, value=1
            ).on_value_change(lambda v: toggle_recoding(v.value == 2))
            with ui.column():
                with ui.row().classes("w-full"):
                    with ui.dropdown_button('Select Model', auto_close=True) as model_selector:
                        ui.item("Detection Off", on_click=lambda: toggle_detection(None))
                        for model in model_list:
                            ui.item(str(model.__name__), on_click=lambda m=model: toggle_detection(m))
                    ui.space()
                    with ui.dropdown_button('Select Controller', auto_close=True) as controller_selector:
                        controller_list
                        ui.item("Control Off", on_click=lambda: toggle_controller(None))
                        for controller in controller_list:
                            ui.item(str(controller.__name__), on_click=lambda m=controller: toggle_controller(m))
                    #do_control = ui.switch('Robot Control', on_change=lambda x: toggle_control(x.value))
                
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
                        ui.button("Go Calib Pose", on_click=lambda x: arm_calib_init())
                        ui.button("Free Joint", on_click=lambda x: arm_calib_free())
                        ui.button("Lock Joint", on_click=lambda x: arm_calib_lock())
                        ui.button("Save Calib Pose", on_click=lambda x: arm_calib_save())
                        # ui.button("Go Home", on_click=lambda x: arm_action_home())
                        # ui.button("Go Calib", on_click=lambda x: arm_action_calib())
                        # ui.button("Go Zero", on_click=lambda x: arm_action_zero())
                        # ui.button("Go Test", on_click=lambda x: arm_action_test())
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
                        with ui.row():
                            btn_store = ui.button("Store Config")
                            btn_load = ui.button("Load Config")
                            ui.button("Refresh", on_click=lambda _: ui_config_panel.refresh())
                        ui_config_panel(robot)

            with splitter.after:
                ui.label("Robot Live View")
                with ui.tabs().classes("w-full") as tabs_view:
                    cam_view_tab_name = "Robot Cam"
                    one_view = ui.tab(cam_view_tab_name)
                    view_ctrl_tab_name = "Controller View"
                    two_view = ui.tab(view_ctrl_tab_name)
                tab_panel_view = ui.tab_panels(tabs_view, value=one_view).classes("w-full")
                with tab_panel_view.classes("w-full h-full"):
                    with ui.tab_panel(one_view):
                        video_image = ui.interactive_image().classes("w-full h-full")
                    with ui.tab_panel(two_view):       
                        controller_plot = ui.line_plot(n=6, limit=300, figsize=(9, 6), update_every=5).with_legend(['Motor1', 'Motor2', 'vel_target', "vel_current", "rot_target", "rot_current"], loc='upper center', ncol=3).classes('w-full h-full border')
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


def update_controller_plot():

    if isinstance(robot.platform, DifferentialDrive):

        plotdata=[]
        plotdata.append([robot.platform.motor_right.get_speed()])
        plotdata.append([robot.platform.motor_left.get_speed()])
        plotdata.append([robot.platform._target_velocity])
        plotdata.append([robot.platform._current_velocity])
        plotdata.append([robot.platform._target_angular_vel])
        plotdata.append([robot.platform._current_angular_vel])
        now = datetime.now()
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            controller_plot.push([now], plotdata) #, y_limits=(-100, 100)
        

timer_controller = ui.timer(0.1, update_controller_plot, active=False)
def tab_panel_view_change():
    if tab_panel_view.value == view_ctrl_tab_name:
        timer_controller.activate()
        live_update_timer.deactivate()

    else:
        timer_controller.deactivate()
        live_update_timer.activate()
tab_panel_view.on_value_change(tab_panel_view_change)



def tab_select_event():
    global live_update_timer, tab_names, video_image
    try:
        if tab_panel.value == tab_names[0]:
            live_update_timer.activate()
            tab_panel_view.set_value(cam_view_tab_name)
        else:
            live_update_timer.deactivate()
            timer_controller.deactivate()
            print("reload files...")
            reload_files()
    except Exception as ex:
        print(ex)
tab_panel.on_value_change(tab_select_event)


def store_config():
    robot.store_properties(robot_config_filename)
    logger.info(f"Robot Config stored as {robot_config_filename}")
btn_store.on_click(store_config)

def load_config():
    robot.load_properties(robot_config_filename)
    ui_config_panel.refresh()
    ui_model_info.refresh()
    model_selector.set_value(None)
    controller_selector.set_value(None)
    print("controller is", robot.get_controller())
    logger.info(f"Robot Config loaded from {robot_config_filename}")

btn_load.on_click(load_config)


if args.cfgload:
    load_config()




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
    robot.stop()
    robot.cleanup()
    await disconnect()

    print("Exit, cleaning up...")
    joystick_end()
    robot.cleanup()
app.on_shutdown(cleanup)


# Start app
ui.run(reload=False, port=8080, show=False, title="Beachbot", favicon=str(get_asset_path() / "beachbot_128x128.png"))
