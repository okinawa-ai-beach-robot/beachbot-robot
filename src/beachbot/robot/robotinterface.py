from enum import Enum
import time
from typing import Any, Union
import numpy as np
import threading
from beachbot.config import logger
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.ai.debrisdetector import DebrisDetector
from beachbot.utils.timer import Timer
from beachbot.utils.videowriteropencv import VideoWriterOpenCV


class RobotInterface(object):
    class CAMERATYPE(Enum):
        FRONT = 1
        GRIPPER = 2

    def __init__(self):
        self.cameradevices={}
        self.platform=None
        self.arm=None

        self.controller=None
        self.controller_lock = threading.Lock()

        self.detector=None
        self.detector_lock = threading.Lock()

        # upper limit for detection loop
        self.ctrl_loop_target_hz=10
        self.ctrl_loop_timer=Timer()

        self.video_writer = None
        self.video_writer_lock = threading.Lock()

        self.buffered_camera_image={}
        self.buffered_camera_image_boxes={}
        self.buffered_camera_lock = threading.Lock()




    def is_recording(self):
        with self.video_writer_lock:
            if self.video_writer is not None:
                return self.video_writer.filename

    def get_detector(self):
        with self.detector_lock:
            return self.detector
        
    def set_detector(self, detector : DebrisDetector = None):
        with self.detector_lock:
            self.detector = detector

    def start_recording(self):
        with self.video_writer_lock:
            if self.video_writer is not None:
                self.video_writer.close()
            capture_width, capture_height = self.cameradevices[RobotInterface.CAMERATYPE.FRONT].get_size()
            self.video_writer = VideoWriterOpenCV(None, fps=self.ctrl_loop_target_hz, capture_width=capture_width, capture_height=capture_height)
            return self.video_writer.filename
    
    def stop_recording(self):
        with self.video_writer_lock:
            lastfile = None
            if self.video_writer is not None:
                lastfile=self.video_writer.filename
                self.video_writer.close()
                self.video_writer=None
            return lastfile


    def get_controller(self) -> RobotController:
        return self.controller

    def set_controller(self, controller : RobotController = None):
        with self.controller_lock: 
            self.controller = controller

    def _run_vision_loop(self):
        while self.vision_thread is not None:
            with self.ctrl_loop_timer:
                which_cam = self.CAMERATYPE.FRONT

                # get frame
                frame = self.get_camera_image(which=which_cam)
                if frame is not None:
                    class_ids=[]
                    confidences=[]
                    boxes=[]
                    boxlist = []
                    with self.detector_lock:
                        if self.detector is not None:
                            # detect boxes
                            class_ids, confidences, boxes = self.detector.apply_model(frame)

                            # format boxes
                            for classid, confidence, box in zip(class_ids, confidences, boxes):
                                if confidence >= 0.01:
                                    boxlist.append(BoxDef(left=box[0], top=box[1], w=box[2], h=box[3], class_name=self.detector.list_classes[classid], confidence=confidence))

                    # store box result for buffered image
                    with self.buffered_camera_lock:
                        self.buffered_camera_image_boxes[which_cam]=boxlist

                    with self.controller_lock:
                        if self.controller is not None:
                            self.controller.update(self, boxlist)
                else:
                    logger.warning("_run_vision_loop: Frame skipped")
            target_interval = 1/self.ctrl_loop_target_hz
            if self.ctrl_loop_timer.get_last_interval()<target_interval:
                # Limit control loop to not exceed self.ctrl_loop_target_hz update rate
                time.sleep(target_interval - self.ctrl_loop_timer.get_last_interval())


    def start(self):
        # deaemon=true, do not wait for thread on exit, just kill it 8-)
        self.vision_thread = threading.Thread(target=self._run_vision_loop, daemon=True) 
        self.vision_thread.start()

    def stop(self):
        ending_thread = self.vision_thread
        self.vision_thread=None
        logger.info("Waiting for robot vision thread to end...")
        ending_thread.join()

    
    def get_buffered_camera_image(self, which:CAMERATYPE=CAMERATYPE.FRONT) -> Union[Any, Union[list[BoxDef], None]] :
        with self.buffered_camera_lock:
            if which in self.buffered_camera_image:
                # return buffered image if available
                return (self.buffered_camera_image[which], self.buffered_camera_image_boxes.get(which, None))
            else:
                # acquire image if not available
                return (self.get_camera_image(which), None)
            
    
    def get_camera_image(self, which:CAMERATYPE=CAMERATYPE.FRONT, stop_others=True):
        res = None
        if stop_others:
            for cameraid in self.cameradevices.keys():
                if which!=cameraid:
                    if self.cameradevices[cameraid].is_running():
                        self.cameradevices[cameraid].stop()

        if which in self.cameradevices.keys():
            if not self.cameradevices[which].is_running():
                self.cameradevices[which].start()
            res = self.cameradevices[which].read()

            if res is not None:
                # buffer most recent camera image
                with self.buffered_camera_lock:
                    self.buffered_camera_image[which]=res
                    self.buffered_camera_image_boxes[which]=None

        return res

    
    def set_target_velocity(self, angular_velocity, velocity, force_overwrite:float=-1):
        self.platform.set_target(angular_velocity, velocity, force_overwrite)

    def set_arm_target(self, joint_angles):
        self.arm.set_joint_targets(joint_angles)
    
    def get_arm_state(self):
        return self.arm.get_joint_state()
    
    def set_gripper_state(self, percent_closed=0):
        self.arm.set_gripper(percent_closed)
    
    def set_arm_active(self, active=True):
        self.arm.set_joints_enabled(active)

    def move_arm_trajectory(self, file):
        data = np.load(file)
        joint_angles = data["qs"]
        taus_recorded = data["taus"]
        time_points = data["ts"].tolist()
        self.arm.replay_trajectory(joint_angles, time_points)

    def move_arm_home(self):
        self.arm.go_home()

    def cleanup(self):
        self.arm.cleanup()
        self.platform.cleanup()
        self.get_camera_image(None)



