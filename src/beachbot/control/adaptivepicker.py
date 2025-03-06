import math
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

        self.setpoint_x = 0.5
        self.setpoint_y = 0.63 # 0.63 is good for current simulation file
        self.register_property("setpoint_x", descr="Horizontal taget position in relative image coordinates, e.g. 0.25 is left quarter of image; 0.5 is image center.")
        self.register_property("setpoint_y", descr="Vertical target position in rleative image coordinates, e.g. 0.25 is lower quarter of image; 0.5 is image center.")

        self.debug_traj_pos=0
        self.register_property("debug_traj_pos", max_value=1.0, min_value=0.0, descr="Arm position in percent of pickup trajectory (only in debug mode)")

        self.debug_offset_x=0
        self.register_property("debug_offset_x", max_value=0.1, min_value=-0.1, descr="Target offset (x, horizontal) in meters (only in debug mode)")

        self.debug_offset_y=0
        self.register_property("debug_offset_y", max_value=0.1, min_value=-0.1, descr="Target offset (y, vertical) in meters (only in debug mode)")

        self.debug_auto_offset=False
        self.register_property("debug_auto_offset", descr="Target offset are estimated according to target_factors, e.g. pixel_dist_x * target_factor_x = offset_x")


        self.target_factor_x=0.8
        self.register_property("target_factor_x", descr="Target offset (x, horizontal) in meters  = image_dist_x * target_factor_x")

        self.target_factor_y=0.8
        self.register_property("target_factor_y", descr="Target offset (y, vertical) in meters = image_dist_y * target_factor_y")

        self.targetfilter=["cup","toilet", "sports ball", "blue_blob"]
        self.register_property("targetfilter", ",".join(self.targetfilter), descr="List of classes, separated by comma; no spaces allowed, class names with space are accepted. E.g. \"cup,sports ball,trash_easy\"")
        


        self.debug_traj_pick = None
        self.debug_traj_toss = None
        self.traj_is_dirty=True


    def property_changed_callback(self, name):
        super().property_changed_callback(name)

        if name=="debug_offset_x" or name=="debug_offset_y":
            # recalculate trajectory to account for offset interpolation
            self.traj_is_dirty=True

            

    def operate_arm(self, robot: RobotInterface):
        logger.info("pickup")
        robot.arm.pickup()
        logger.info("tossing")
        robot.arm.toss()
        robot.arm.go_home()

    def autoset_target_offset(self, robot: RobotInterface, detections: List[BoxDef] = None):

        # trash_to_follow is a list of detections with easy sorting based on BoxDef.confidence
        # It should only contain objects that match the targetfilter
        trash_to_pick: List[BoxDef] = []
        for det in detections:
            if det.class_name in self.targetfilter:
                
                trash_x = best_match.left+best_match.w/2
                trash_y = 1.0 - (best_match.top+best_match.h/2) # 0 is bottom, 1 is top
                
                error_x = self.setpoint_x - trash_x
                error_y = self.setpoint_y - trash_y
                dist = math.sqrt((error_x)**2 + (error_y)**2)
                det.err = (error_x, error_y)
                det.dist=dist
                trash_to_pick.append(det)
        if trash_to_pick is not None and len(trash_to_pick) > 0:
            # sort by confidence
            trash_to_pick.sort(key=lambda x: x.dist, reverse=False)
            # approach trash
            best_match = trash_to_pick[0]

            return best_match
        return None


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
        
        elif self.arm_thread is None:
            # if in debug mode, read properties to control arm for testing:
            if self.traj_is_dirty==True and not self.debug_auto_offset:
                self.traj_is_dirty=False
                logger.debug(f"Recalculate pickup trajectory with offsets {(self.debug_offset_x, self.debug_offset_y)}")
                self.debug_traj_pick = robot.arm._interpolate_traj("pickup", (self.debug_offset_x, self.debug_offset_y))
                self.debug_traj_toss = robot.arm._interpolate_traj("toss", (self.debug_offset_x, self.debug_offset_y))
                logger.debug(f"Loaded tajectory lengths are {self.debug_traj_pick.get_length()} and {self.debug_traj_toss.get_length()}")

            if self.debug_auto_offset and (self.debug_traj_pos<0.25 or self.traj_is_dirty):
                self.traj_is_dirty=False
                # auto estimate the trajectory offsets:
                det = self.autoset_target_offset(robot, detections)
                if det is not None:
                    offsets = (min(0.1, max(-0.1,det.err[0]*-self.target_factor_x)), min(0.1, max(-0.1,det.err[1]*-self.target_factor_y)))
                    logger.debug(f"Auto estimate pickup trajectory with offsets {offsets}, err is {det.err}")
                    self.debug_traj_pick = robot.arm._interpolate_traj("pickup", offsets)
                    self.debug_traj_toss = robot.arm._interpolate_traj("toss", offsets)



            if self.debug_traj_pos<0.5:
                # pickup
                qentry = round((self.debug_traj_pick.get_length()-1) * self.debug_traj_pos/0.5)
                robot.arm.set_joint_targets(self.debug_traj_pick.qs[qentry])
            else:
                # toss
                qentry = round((self.debug_traj_toss.get_length()-1) * (self.debug_traj_pos-0.5)/0.5)
                robot.arm.set_joint_targets(self.debug_traj_toss.qs[qentry])

            return RESULT.BUSY
        elif self.arm_thread is not None:
            if not self.arm_thread.is_alive():
                # Done, we are not busy anymore with pickup, delete thread for arm movement
                self.arm_thread = None
        
        return RESULT.BUSY


            
        

