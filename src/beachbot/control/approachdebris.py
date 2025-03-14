import math
import time
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.utils.controllercollection import PIDController
from beachbot.config import logger


class ApproachDebris(RobotController):
    def __init__(self, parent:RobotController = None):
        super().__init__()
        self.minimum_output = 10
        # Used for a basic hysteresis filter
        self.missing_target_count = 0
        self.target_arrival_frames = 0

        # Default values of pid controller gain horizontal/vertical, i.e. for rotaitonal/distance difference
        default_kp_x = 160.0
        default_kp_y = 160.0

        # Initial target setpoints, in relative position (0..1)
        default_setpoint_x = 0.5
        # default_setpoint_y = 0.62 # 0.62 is good for current simulation file
        default_setpoint_y = 0.47 # good for Jetson camera


        # Output estimated control to motors
        self.output_enabled=False
        self.register_property("output_enabled")
        self.register_property("setpoint_x", default_setpoint_x)
        self.register_property("setpoint_y", default_setpoint_y)
        
        self.pid_error_threshold_x = 0.02 # with the adaptive gripper, we can be more "free" here as a default
        self.pid_error_threshold_y = 0.02
        self.register_property("pid_error_threshold_x", descr="Decide if target is reached if horizontal errors are below this threshold, coordinates in relative position (0..1)")
        self.register_property("pid_error_threshold_y", descr="Decide if target is reached if vertical errors are below this threshold, coordinates in relative position (0..1)")
        self.pid_debug=False

        # Targetfilter: list of target classes to follow, e.g. "trash_easy,trash_hard":
        if parent is not None and parent.get_property('targetfilter') is not None:
            # Parent has targetfilter list
            self.targetfilter_source = parent
        else:
            # Parent class does not have targetfilter, save property in this class, populate default variables
            self.targetfilter_source = self
            self.targetfilter=["cup","bottle", "trash_easy", "sports ball", "blue_blob"]
            self.register_property("targetfilter", ",".join(self.targetfilter), descr="List of classes, separated by comma; no spaces allowed, class names with space are accepted. E.g. \"cup,sports ball,trash_easy\"")

        # Register controller parameters in gui, property changes computet in "property_changed_callback"
        self.register_property("kp_x", default_value=default_kp_x, descr="PID controller proportial gain for horizontal/rotational errors")
        self.register_property("kp_y", default_value=default_kp_y, descr="PID controller proportial gain for vertical/distance errors")
        self.register_property("setpoint_x", default_setpoint_x, descr="Horizontal taget position in relative image coordinates, e.g. 0.25 is left quarter of image; 0.5 is image center.")
        self.register_property("setpoint_y", default_setpoint_y, descr="Vertical target position in rleative image coordinates, e.g. 0.25 is lower quarter of image; 0.5 is image center.")

        # Register member variables of this class, will be update automatically in case of user interaction        
        self.register_property("output_enabled", descr="Output estimated control values to motors.")
        self.register_property("pid_debug")

        
        
        # Create pid controller:
        self.ctrl = PIDController(setpoint_x=default_setpoint_x, setpoint_y=default_setpoint_y, kp=(default_kp_x, default_kp_y))

    def property_changed_callback(self, name):
        # Setpoints and gains have to be set in the ctrl class, other properties are set in this class as default
        if name=="setpoint_x":
            self.ctrl.setpoint_x=self.get_property(name)
        elif name=="setpoint_y":
            self.ctrl.setpoint_y=self.get_property(name)
        elif name=="kp_x":
            self.ctrl.kp=(self.get_property(name),self.ctrl.kp[1])
        elif name=="kp_y":
            self.ctrl.kp=(self.ctrl.kp[0], self.get_property(name))
        else:
            super().property_changed_callback(name)
            

    def update(self, robot: RobotInterface, detections: List[BoxDef]=None) -> bool:
        """
        Approach trash

        Args:
            robot (RobotInterface): Robot interface
            detections (List[BoxDef], optional): List of detections. Defaults to None.

        Returns:
            bool: True if controller has acheived target, False otherwise 
        """

        # trash_to_follow is a list of detections with easy sorting based on BoxDef.confidence
        # It should only contain objects that match the targetfilter
        trash_to_follow: List[BoxDef] = []
        for det in detections:
            if det.class_name in self.targetfilter_source.targetfilter:
                trash_to_follow.append(det)

        if trash_to_follow is not None and len(trash_to_follow) > 0:
            self.missing_target_count = 0
            # sort by confidence
            trash_to_follow.sort(key=lambda x: x.confidence, reverse=True)
            # approach trash
            best_match = trash_to_follow[0]
            trash_x = best_match.left+best_match.w/2
            trash_y = 1.0 - (best_match.top+best_match.h/2) # 0 is bottom, 1 is top

            if self.debug:
                print("trash position:", trash_x, trash_y)

            dir_command = self.ctrl.get_output(trash_x, trash_y, self.pid_debug)
            dir_error_x = self.ctrl.prev_error_x
            dir_error_y = self.ctrl.prev_error_y

            # Ignore low outputs to prevent motor stalling
            dir_command = [min(max(n, -100), -self.minimum_output) if n < -self.minimum_output else
                           max(min(n, 100), self.minimum_output) if n > 15 else 0
                           for n in dir_command]

            if self.debug:
                logger.debug(f"dir_command: {dir_command}, dir_error: {dir_error_x, dir_error_y} target_arrival_frames: {self.target_arrival_frames}")

            if self.output_enabled:
                robot.set_target_velocity(-dir_command[0], -dir_command[1])
            else:
                robot.set_target_velocity(0,0)

            if abs(dir_error_x) < self.pid_error_threshold_x and abs(dir_error_y) < self.pid_error_threshold_y:
                self.target_arrival_frames += 1
                if self.target_arrival_frames > 10:
                    robot.set_target_velocity(0, 0)
                    logger.info("ApproachDebris: Target reached")
                    return RESULT.SUCCESS
            else:
                self.target_arrival_frames = 0

        else:
            self.target_arrival_frames = 0
            self.missing_target_count += 1

            if self.debug:
                print("could not see anything!", self.missing_target_count)
            
            if self.missing_target_count > 10:
                # Rotate robot, TODO add a 3rd controller for "random seach"
                # TODO return RESULT.FAILURE to indicate controller selector to handle the situation appropriately
                # for the controllerselector to check if (1) approaching, (2) reached, (3) Lost
                if self.output_enabled:
                    robot.set_target_velocity(angular_velocity=0, velocity=0)


        return RESULT.BUSY
