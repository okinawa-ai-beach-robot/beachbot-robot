import math
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.utils.controllercollection import PIDController
from beachbot.config import logger
import time

from jaraco.context import robust_temp_dir


class RoamAround(RobotController):
    def __init__(self):
        super().__init__()

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None) -> bool:
        trash_to_follow: List[BoxDef] = []
        for det in detections:
            if det.class_name in self.targetfilter:
                trash_to_follow.append(det)
        t_end = time.time() + 10
        while time.time() < t_end:
            # test how much time does the robot finish a loop
            robot.set_target_velocity(0, 2*math.pi)
            if trash_to_follow is not None:
                robot.stop()
                return RESULT.SUCCESS
        robot.set_target_velocity(0,math.pi/2)
        self.roam()


    # set the robot to roam with paht with
    def roam (self,robot: RobotInterface):
        for i in range(4):
            robot.set_target_velocity(15, 0)
            robot.set_target_velocity(0,math.pow(-1,i)*math.pi/2)
            robot.set_target_velocity(5,0)
            robot.set_target_velocity(0,math.pow(-1,i)*math.pi/2)




