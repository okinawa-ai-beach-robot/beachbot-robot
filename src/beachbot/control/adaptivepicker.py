import math
from threading import Thread
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef 
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.config import logger
from beachbot.utils.timer import Timer


class AdaptivePickupController(RobotController):
    def __init__(self):
        super().__init__()
        self.arm_thread=None
        self.stable_time=1.0 # one second object observation without interruption before picking up
        self.timout_time=10.0 # if finding sable object takes mote than 20 seconds, abort ... 

        self.manual_mode=False
        self.register_property("manual_mode", descr="Do not pickup automatically, wait for user properties changes")

        self.setpoint_x = 0.5
        #self.setpoint_y = 0.63 # 0.63 is good for current simulation file
        self.setpoint_y = 0.26 # 0.28 is good for robot lowr edge
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
        
        self.arm_speedfactor = 5
        self.register_property("arm_speedfactor", min_value=5, max_value=30, descr="Speed factor, adjusting of robot arm movement speed.")


        self.debug_traj_pick = None
        self.debug_traj_toss = None
        self.traj_is_dirty=True

        self.stable_timer = None
        self.timeout_timer = None


    def property_changed_callback(self, name):
        super().property_changed_callback(name)

        if name=="debug_offset_x" or name=="debug_offset_y":
            # recalculate trajectory to account for offset interpolation
            self.traj_is_dirty=True

            

    def operate_arm(self, robot: RobotInterface, offset=(0,0)):
        logger.debug(f"Recalculate pickup trajectory with offsets {offset}")
        traj_pick = robot.arm._interpolate_traj("pickup", offset)
        traj_toss = robot.arm._interpolate_traj("toss", offset)
        logger.info("Execute: pickup")
        # TODO for now do not use interpolated trajectories, recordings depend on os time (estimate proper sim time is todo)
        robot.arm.replay_trajectory(traj_pick.qs, robot.arm.pickup_trajectory.ts, speed_factor=self.arm_speedfactor)
        logger.info("Execute: tossing")
        robot.arm.replay_trajectory(traj_toss.qs, robot.arm.toss_trajectory.ts, speed_factor=self.arm_speedfactor)
        robot.arm.go_home()

    def autoset_target_offset(self, robot: RobotInterface, detections: List[BoxDef] = None):

        # trash_to_follow is a list of detections with easy sorting based on BoxDef.confidence
        # It should only contain objects that match the targetfilter
        trash_to_pick: List[BoxDef] = []
        for det in detections:
            if det.class_name in self.targetfilter:
                
                trash_x = det.left+det.w/2
                trash_y = 1.0 - (det.top+det.h) # 0 is bottom, 1 is top, trash lower y position is position on ground!

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
    
    def _cleanup(self):
        """
        Reset Controller to inital state (start sesuring time from scratch next time update is called)
        """
        self.timeout_timer=None
        self.stable_timer=None

   



    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        # First identify position of object to pick (expected to be close to (setpoint_x,setpoint_y)
        det = self.autoset_target_offset(robot, detections)



        # check if arm-thread is running, wait for completion, return be BUSY until thread finishes
        if self.arm_thread is not None:
            if not self.arm_thread.is_alive():
                # Done, we are not busy anymore with pickup, delete thread for arm movement
                self.arm_thread = None

                self._cleanup()
                #expect that object disappeared for success:
                if det is None:
                    return RESULT.SUCCESS
                else:
                    # Object still in visual field after pick-up, Retry?
                    return RESULT.FAILURE
            else:
                return RESULT.BUSY
            
        



        if det is not None:
            # Estimate pickup offset
            offsets = (det.err[0]*-self.target_factor_x, det.err[1]*-self.target_factor_y)

            # Is the object reachable?
            obj_is_reachable = (offsets[0]>=-0.1 and offsets[0]<=0.1 and offsets[1]>=-0.1 and offsets[1]<=0.1)

            # bound reaching offset to valid ones
            offsets = (min(0.1, max(-0.1,det.err[0]*-self.target_factor_x)), min(0.1, max(-0.1,det.err[1]*-self.target_factor_y)))
        else:
            # No object detected:
            offsets=(0,0)
            obj_is_reachable=False
            offsets=(0,0)


        if not self.manual_mode:
            # Normal operation, initiate pickup sequence
            # Create thread for arm movement, return, to not block controller loop


            # Measure time since first call of controller
            if self.timeout_timer is None:
                self.timeout_timer = Timer().start()
            if self.stable_timer is None:
                self.stable_timer = Timer().start()


            # if det is None or not reachable, wait for a certain amount of time
            # Restart the timer counting time since first appearance of object
            if det is None or not obj_is_reachable:
                self.stable_timer.start()


            if self.timeout_timer.measure()>self.timout_time:
                # timeout during finding object occured, abort.
                if self.debug:
                    if det is not None:
                        logger.info(f"Adaptive pickup controller can not reach object at {det.obj_pos} with pickup offsets {offsets}")
                    else:
                        logger.info("Adaptive Picker could not find object (timeout)")
                self._cleanup()
                return RESULT.FAILURE
            


                
            if self.stable_timer.measure()>self.stable_time:
                # We observed target for a certain amount of time, without interruption.
                # Proceed with pickup
                logger.debug(f"Initiate pickup of object at {det.obj_pos} with pickup offsets {offsets}")
                self.arm_thread = Thread(target=lambda r=robot:self.operate_arm(robot=r, offset=offsets))
                self.arm_thread.start()
                self._cleanup()
                # Pickup thread in progress, indicate busy controller
            return RESULT.BUSY



        
        else:
            # Debug mode, manual operation

            # 2.1 Trajectory estimation in manual mode can be automatic or based on slider values
            if self.traj_is_dirty==True and not self.debug_auto_offset:
                #Manual estimation of offsets
                self.traj_is_dirty=False
                if self.debug:
                    logger.debug(f"Recalculate pickup trajectory with offsets {(self.debug_offset_x, self.debug_offset_y)}")
                self.debug_traj_pick = robot.arm._interpolate_traj("pickup", (self.debug_offset_x, self.debug_offset_y))
                self.debug_traj_toss = robot.arm._interpolate_traj("toss", (self.debug_offset_x, self.debug_offset_y))
            elif (self.debug_auto_offset and self.debug_traj_pos<0.25) or self.debug_traj_pick is None or self.debug_traj_toss is None:
                # Automatic estimation of offset only at the beginning of trajectory as later the object is covered by the gripper
                if det is not None and self.debug:
                    logger.debug(f"Auto estimate pickup trajectory with offsets {offsets}, err is {det.err}, pos is {det.obj_pos}")
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
            self._cleanup()
            return RESULT.BUSY


            
        

