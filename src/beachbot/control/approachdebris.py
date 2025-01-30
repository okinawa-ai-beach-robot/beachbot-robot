from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.utils.controllercollection import PIDController
from beachbot.config import logger


class ApproachDebris(RobotController):
    def __init__(self):
        super().__init__()

        default_kp = 100.0
        default_setpoint_x = 0.5
        default_setpoint_y = 0.25
        detection_threshold = 0.5
        # Used for a basic hysteresis filter
        self.missing_target_count = 0

        self._register_property("kp", default_kp)
        self._register_property("setpoint_x", default_setpoint_x)
        self._register_property("setpoint_y", default_setpoint_y)
        self._register_property("detection_threshold", detection_threshold)
        self._register_property("setpoint_y", default_setpoint_y)

        self.targetfilter=["bottle"]
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

    def update(self, robot: RobotInterface, detections: List[BoxDef]=None, debug=False) -> bool:
        """
        Approach trash

        Args:
            robot (RobotInterface): Robot interface
            detections (List[BoxDef], optional): List of detections. Defaults to None.
            debug (bool, optional): Debug mode. Defaults to False.

        Returns:
            bool: True if controller has acheived target, False otherwise 
        """
        if detections is None:
            return

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

            if debug:
                print("trash position:", trash_x, trash_y)

            dir_command = self.ctrl.get_output(trash_x, trash_y)
            if debug:
                print("dir_command:", dir_command)
            robot.set_target_velocity(-dir_command[0], -dir_command[1])
            error_x = self.ctrl.prev_error_x
            error_y = self.ctrl.prev_error_y
            if error_x < 0.01 and error_y < 0.01:
                robot.set_target_velocity(0, 0)
                logger.info("ApproachDebris: Target reached")
                return True

        else:
            self.missing_target_count += 1
            if self.missing_target_count > 10:
                robot.platform.motor_left.change_speed(50)
                robot.platform.motor_right.change_speed(-50)

        return False
