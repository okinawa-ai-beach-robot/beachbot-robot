from threading import Thread
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef 
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.config import logger


class AdaptivePickupController(RobotController):
    def __init__(self):
        super().__init__()
        self.arm_thread=None

        self.debug_traj_pos=0
        self.register_property("debug_traj_pos", max_value=0.1, min_value=-0.1, descr="Arm position in percent of pickup trajectory (only in debug mode)")

        self.debug_offset_x=0
        self.register_property("debug_offset_x", max_value=0.1, min_value=-0.1, descr="Target offset (x, horizontal) in meters (only in debug mode)")

        self.debug_offset_y=0
        self.register_property("debug_offset_y", max_value=0.1, min_value=-0.1, descr="Target offset (y, vertical) in meters (only in debug mode)")

    def operate_arm(self, robot: RobotInterface):
        logger.info("pickup")
        robot.arm.pickup()
        logger.info("tossing")
        robot.arm.toss()
        robot.arm.go_home()


    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if not self.debug:
            # If not debug, default arm behaviour is executed:
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
        
        else:
            # if in debug mode, read properties to control arm for testing:
            print("yeaha!")
            return RESULT.BUSY
        

