import math
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.utils.controllercollection import PIDController
from beachbot.config import logger


class ApproachDebris(RobotController):
    def __init__(self):
        super().__init__()

        
        
        # Used for a basic hysteresis filter
        self.missing_target_count = 0
        self.target_arrival_frames = 0

        default_kp = 160.0
        default_setpoint_x = 0.5
        default_setpoint_y = 0.5

        self.output_enabled=False
        self.register_property("output_enabled")

        self.register_property("kp", default_kp)
        self.register_property("setpoint_x", default_setpoint_x)
        self.register_property("setpoint_y", default_setpoint_y)

        self.detection_threshold = 0.5
        self.register_property("detection_threshold")
        self.pid_error_threshold = 0.05
        self.pid_debug=False
        self.register_property("pid_debug")

        self.targetfilter=["cup","toilet"]
        # targetfilter: list of target classes to follow, e.g. "trash_easy,trash_hard":
        self.register_property("targetfilter", ",".join(self.targetfilter))
        self.ctrl = PIDController(setpoint_x=default_setpoint_x, setpoint_y=default_setpoint_y, kp=default_kp)

    def property_changed_callback(self, name):

        # Setpoints have to be set in the ctrl class, other properties are set in this class as default
        if name=="setpoint_x":
            self.ctrl.setpoint_x=self.get_property(name)
        elif name=="setpoint_y":
            self.ctrl.setpoint_y=self.get_property(name)
        elif name=="kp":
            self.ctrl.kp=self.get_property(name)
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
            if det.class_name in self.targetfilter:
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

            if self.debug:
                print("dir_command:", dir_command)
                print("dir_error:", (dir_error_x, dir_error_y))

            if self.output_enabled:
                robot.set_target_velocity(-dir_command[0], -dir_command[1])
            else:
                robot.set_target_velocity(0,0)

            if abs(dir_error_x) < self.pid_error_threshold and abs(dir_error_y) < self.pid_error_threshold:
                self.target_arrival_frames += 1
                if self.target_arrival_frames > 20:
                    robot.set_target_velocity(0, 0)
                    logger.info("ApproachDebris: Target reached")
                    return True

        else:
            self.missing_target_count += 1
            if self.missing_target_count > 10:
                # Rotate robot, TODO add a 3rd controller for "random seach"
                # But requires a return value more than True/False, maybe string? or other way
                # for the controllerseelctor to check if (1) approaching, (2) reached, (3) Lost
                if self.output_enabled:
                    robot.set_target_velocity(angular_velocity=0, velocity=0)


        return False
