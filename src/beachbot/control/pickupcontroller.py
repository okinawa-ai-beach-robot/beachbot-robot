import time
from threading import Thread
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef 
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.config import logger


class PickupController(RobotController):
    def __init__(self, parent:RobotController = None):
        super().__init__()
        self.arm_thread=None

    def operate_arm(self, robot: RobotInterface):
        logger.info("pickup")
        robot.arm.pickup()
        logger.info("tossing")
        robot.arm.toss()
        current_qs = robot.arm.get_joint_angles()
        current_qs[3] += -5
        current_qs[2] += -30
        robot.arm.set_joint_targets(current_qs)
        time.sleep(1)
        robot.arm.open_gripper()
        robot.arm.go_home()


    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if self.arm_thread is None:
            # Create thread for arm movement, return, to not block controller loop
            self.arm_thread = Thread(target=lambda r=robot:self.operate_arm(robot=r))
            self.arm_thread.start()
            return RESULT.BUSY
        elif self.arm_thread.is_alive():
            # Thread for arm movement is still running, so we are still busy
            return RESULT.BUSY


        # Done, we are not busy anymore with pickup, delete thread for arm movement
        self.arm_thread = None
        #TODO assess situation and adjust return value appropriately, was pickup successful?
        return RESULT.SUCCESS
