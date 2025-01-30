from typing import List

from ..robot.robotinterface import RobotInterface
from .robotcontroller import RobotController, BoxDef
from ..utils.controllercollection import PIDController


class ApproachDebris(RobotController):
    def __init__(self):
        super().__init__()

        default_kp = 0.0
        default_setpoint_x = 0.5
        default_setpoint_y = 0.25
        detection_threshold = 0.5

        self._register_property("kp", default_kp)
        self._register_property("setpoint_x", default_setpoint_x)
        self._register_property("setpoint_y", default_setpoint_y)
        self._register_property("detection_threshold", detection_threshold)
        self._register_property("setpoint_y", default_setpoint_y)


        self.targetfilter=["chair"]
        # targetfilter: list of target classes to follow, e.g. "trash_easy,trash_hard":
        self._register_property("targetfilter", ",".join(self.targetfilter))


        

        self.ctrl = PIDController(setpoint_x=default_setpoint_x, setpoint_y=default_setpoint_y, kp=default_kp)

    

    def _property_changed_callback(self, name):
        if name=="kp":
            self.ctrl.kp = self.get_property(name)
        elif name=="setpoint_x":
            self.ctrl.setpoint_x=self.get_property(name)
        elif name=="setpoint_y":
            self.ctrl.setpoint_y=self.get_property(name)
        elif name=="targetfilter":
            self.targetfilter = self.get_property(name).split(",")
        elif name=="detection_threshold":
            print("detection_threshold", self.get_property(name))
        else:
            super()._property_changed_callback(name)

    def update(self, robot: RobotInterface, detections: List[BoxDef]=None, debug=False):
        trash_to_follow : BoxDef = None

        for detected in detections:
            # Detected trash
            # Todo select best trash in case multiple detections, for now, take first one:
            if self.targetfilter is None or detected.class_name in self.targetfilter:
                trash_to_follow = detected

        if trash_to_follow:
            # approach trash
            trash_x = trash_to_follow.left+trash_to_follow.w/2
            trash_y = 1.0 - (trash_to_follow.top+trash_to_follow.h/2) # 0 is bottom, 1 is top

            if debug:
                print("trash position:", trash_x, trash_y)

            dir_command = self.ctrl.get_output(trash_x, trash_y)
            if debug:
                print("dir_command:",dir_command)
            robot.set_target_velocity(-dir_command[0], -dir_command[1])



        else:
            # Do random movements to find a trash:
            pass
        
        
