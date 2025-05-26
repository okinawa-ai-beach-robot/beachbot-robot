import math
import random
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.utils.controllercollection import PIDController
from beachbot.config import logger
import time

from charset_normalizer import detect
from jaraco.context import robust_temp_dir


class RoamAround(RobotController):
    def __init__(self):
        super().__init__()
        self.recentTime1=-1
        self.timeLength1=-1
        self.recentTime2=-1
        self.timeLength2=-1
        self.output_enabled = False
        self.targetfilter=[]
        self.blockfilter=[]
        self.register_property("output_enabled", descr="Output estimated control values to motors.")
        self.register_property("targetfilter", ",".join(self.targetfilter),
                               descr="List of classes, separated by comma; no spaces allowed, class names with space are accepted. E.g. \"cup,sports ball,trash_easy\"")
        self.register_property("blockfilter", ",".join(self.blockfilter),
                               descr="List of classes, separated by comma; no spaces allowed, class names with space are accepted. E.g. \"cup,sports ball,trash_easy\"")

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None) -> bool:
        trash_to_follow: List[BoxDef] = []
        if self.output_enabled:
            if self.aviodence(robot, detections) is False:
                if self.recentTime1 == -1:
                    self.recentTime1 = time.time()
                    self.timeLength1=random.randint(1, 5)
                if time.time() - self.recentTime1 < self.timeLength1:
                    robot.set_target_velocity(31.4, 0)
                    logger.info(detections)
                    for det in detections:
                        if det.class_name in self.targetfilter:
                            robot.set_target_velocity(0, 0)
                            self.recentTime1 = -1
                            self.timeLength1 = -1
                            return RESULT.SUCCESS
                else:
                    if self.recentTime2 == -1:
                        self.recentTime2 = time.time()
                        self.timeLength2 = random.randint(10, 20)
                    if time.time() - self.recentTime2 < self.timeLength2:
                        robot.set_target_velocity(0, 10)
                        logger.info(detections)
                        for det in detections:
                            if det.class_name in self.targetfilter:
                                robot.set_target_velocity(0, 0)
                                self.recentTime1 = -1
                                self.timeLength1 = -1
                                self.recentTime2 = -1
                                self.timeLength2 = -1
                                return RESULT.SUCCESS
                    else:
                        self.recentTime1 = -1
                        self.timeLength1 = -1
                        self.recentTime2 = -1
                        self.timeLength2 = -1

    def aviodence(self,robot: RobotInterface,detctions):
        trash_to_aviod: List[BoxDef] = []
        for dec in detctions:
            if dec.class_name in self.blockfilter:
                trash_to_aviod.append(dec)
        for trash in trash_to_aviod:
            trash_y = 1.0 - (trash.top+trash.h/2)
            if  trash_y < 0.1:
                robot.set_target_velocity(31.4, 0)
                return True
        return False






