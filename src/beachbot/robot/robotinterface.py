from enum import Enum
import numpy as np

class RobotInterface(object):
    class CAMERATYPE(Enum):
        FRONT = 1
        GRIPPER = 2

    def __init__(self):
        self.cameradevices={}
        self.platform=None
        self.arm=None

    def stop(self):
        raise NotImplementedError()
    
    def get_camera_image(self, which:CAMERATYPE=CAMERATYPE.FRONT, stop_others=True):
        """Take snaphot of camera device.

        Args:
            which (CAMERATYPE, optional): Which Camera to take snapshot from. Defaults to CAMERATYPE.FRONT.
            stop_others (bool, optional): Stop recording of all other cameras (make others inactive). Defaults to True.

        Returns:
            [numpy.Array|None]: Most recent camera snapshot
        """
        res = None
        if stop_others:
            for cameraid in self.cameradevices.keys():
                if which!=cameraid:
                    if self.cameradevices[cameraid].is_running():
                        self.cameradevices[cameraid].stop()

        if which in self.cameradevices.keys():
            if not self.cameradevices[which].is_running():
                self.cameradevices[which].start()
            res = self.cameradevices[which].read()

        return res

    
    def set_target_velocity(self, angular_velocity, velocity):
        self.platform.set_target(angular_velocity, velocity)

    def set_arm_target(self, joint_angles):
        self.arm.set_joint_targets(joint_angles)
    
    def get_arm_state(self):
        return self.arm.get_joint_state()
    
    def set_gripper_state(self, percent_closed=0):
        self.arm.set_gripper(percent_closed)
    
    def set_arm_active(self, active=True):
        self.arm.set_joints_enabled(active)

    def move_arm_trajectory(self, file):
        data = np.load(file)
        joint_angles = data["qs"]
        taus_recorded = data["taus"]
        time_points = data["ts"].tolist()
        self.arm.replay_trajectory(joint_angles, time_points)

    def move_arm_home(self):
        self.arm.go_home()

    def cleanup(self):
        self.arm.cleanup()
        self.platform.cleanup()
        self.get_camera_image(None)



