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
        self.pos_waitcounter=10
        self.timeoutcounter=100

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

            

    def operate_arm(self, robot: RobotInterface, offset=(0,0), speed_factor=20):
        logger.debug(f"Recalculate pickup trajectory with offsets {offset}")
        traj_pick = robot.arm._interpolate_traj("pickup", offset)
        traj_toss = robot.arm._interpolate_traj("toss", offset)
        logger.info("Execute: pickup")
        robot.arm.replay_trajectory(traj_pick.qs, traj_pick.ts, speed_factor=speed_factor)
        logger.info("Execute: tossing")
        robot.arm.replay_trajectory(traj_toss.qs, traj_toss.ts, speed_factor=speed_factor)
        robot.arm.go_home()

    def autoset_target_offset(self, robot: RobotInterface, detections: List[BoxDef] = None):

        # trash_to_follow is a list of detections with easy sorting based on BoxDef.confidence
        # It should only contain objects that match the targetfilter
        trash_to_pick: List[BoxDef] = []
        for det in detections:
            if det.class_name in self.targetfilter:
                
                trash_x = det.left+det.w/2
                trash_y = 1.0 - (det.top+det.h/2) # 0 is bottom, 1 is top

                det.obj_pos=(trash_x, trash_y)
                
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
        # First identify position of object to pick (expected to be close to (setpoint_x,setpoint_y)
        det = self.autoset_target_offset(robot, detections)



        # check if arm-thread is running, wait for completion, return be BUSY until thread finishes
        if self.arm_thread is not None:
            if not self.arm_thread.is_alive():
                # Done, we are not busy anymore with pickup, delete thread for arm movement
                self.arm_thread = None

                #expect that object disappeared for success:
                if det is None:
                    return RESULT.SUCCESS
                else:
                    # Object still in visual field after pick-up, Retry?
                    return RESULT.FAILURE
            else:
                return RESULT.BUSY
            
        


        if det is None:
            # No object in sight, can not pick up!
            return RESULT.FAILURE
        
        # Estimate pickup offset
        offsets = (det.err[0]*-self.target_factor_x, det.err[1]*-self.target_factor_y)

        # Is the object reachable?
        obj_is_reachable = (offsets[0]>=-0.1 and offsets[0]<=0.1 and offsets[1]>=-0.1 and offsets[1]<=0.1)

        # bound reaching offset to valid ones
        offsets = (min(0.1, max(-0.1,det.err[0]*-self.target_factor_x)), min(0.1, max(-0.1,det.err[1]*-self.target_factor_y)))

        if not self.debug:
            # Normal operation, initiate pickup sequence
            # Create thread for arm movement, return, to not block controller loop

            # Wait maximum of self.timeoutcounter steps to get ready for pickup, otherwise return failure
            self.timeoutcounter -= 1
            if self.timeoutcounter<1:
                logger.info(f"Adaptive pickup controller can not reach object at {det.obj_pos} with pickup offsets {offsets}")
                self.timeoutcounter = 100
                return RESULT.FAILURE
            
            if not obj_is_reachable:
                # can not reach object with arm
                # reset waittime
                self.pos_waitcounter=10
                return RESULT.BUSY
            
            if self.pos_waitcounter>0:
                self.pos_waitcounter -= 1
            else:
                # if object was pickable for self.pos_waitcounter steps (stable) in a row, pick it up!
                self.pos_waitcounter=10
                self.timeoutcounter = 100
                logger.debug(f"Initiate pickup of object at {det.obj_pos} with pickup offsets {offsets}")
                self.arm_thread = Thread(target=lambda r=robot:self.operate_arm(robot=r, offset=offsets))
                self.arm_thread.start()
            return RESULT.BUSY
        
        else:
            # Debug mode, manual operation

            # 2.1 Trajectory estimation in manual mode can be automatic or based on slider values
            if self.traj_is_dirty==True and not self.debug_auto_offset:
                #Manual estimation of offsets
                self.traj_is_dirty=False
                logger.debug(f"Recalculate pickup trajectory with offsets {(self.debug_offset_x, self.debug_offset_y)}")
                self.debug_traj_pick = robot.arm._interpolate_traj("pickup", (self.debug_offset_x, self.debug_offset_y))
                self.debug_traj_toss = robot.arm._interpolate_traj("toss", (self.debug_offset_x, self.debug_offset_y))
            elif self.debug_auto_offset and self.debug_traj_pos<0.25:
                # Automatic estimation of offset only at the beginning of trajectory as later the object is covered by the gripper
                logger.debug(f"Auto estimate pickup trajectory with offsets {offsets}, err is {det.err}")
                self.debug_traj_pick = robot.arm._interpolate_traj("pickup", offsets)
                self.debug_traj_toss = robot.arm._interpolate_traj("toss", offsets)


            # 2.2 Read position of trajectory from slider, set joint angles accordingly
            if self.debug_traj_pos<0.5:
                # pickup
                qentry = round((self.debug_traj_pick.get_length()-1) * self.debug_traj_pos/0.5)
                robot.arm.set_joint_targets(self.debug_traj_pick.qs[qentry])
            else:
                # toss
                qentry = round((self.debug_traj_toss.get_length()-1) * (self.debug_traj_pos-0.5)/0.5)
                robot.arm.set_joint_targets(self.debug_traj_toss.qs[qentry])


            # Reset waitcounter in case we switch back to automatic mode:
            self.timeoutcounter = 100
            self.pos_waitcounter = 10
            return RESULT.BUSY


            
        

