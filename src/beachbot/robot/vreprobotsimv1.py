import os
from ..sensors.vrepcamerasim import VrepCameraSim
from ..manipulators.drive import DifferentialDrive
from ..manipulators.vrepmotorsim import VrepMotorSim
from ..manipulators.vreproarmm1sim import VrepRoArmM1Sim
from .robotinterface import RobotInterface
from ..config import config
from ..utils.vrepsimulation import vrep
from coppeliasim_zmqremoteapi_client import *
from pathlib import Path
from beachbot.utils.github_api import download as github_download

class VrepRobotSimV1(RobotInterface):
    def __init__(self, scene=None):
        super().__init__()

        self._base_folder_sim = config.BEACHBOT_SIMULATION

        self.scene_path=self._base_folder_sim / scene

        # TODO Update pr2 branch when merged
        github_download(self.scene_path,
                        config.BEACHBOT_HARDWARE_REPO,
                        "models/coppeliasim/" + scene,
                        "pr2",
                        overwrite=True
                        )

        # Simulator Setup:
        self._vrep_init(scene)

        # Camera Setup:
        self.cameradevices[RobotInterface.CAMERATYPE.FRONT] = VrepCameraSim(self._vrep_sim, "cam_front")

        # Motor Controller Setup:
        motor_left = VrepMotorSim(self._vrep_sim, "motor_left")
        motor_right = VrepMotorSim(self._vrep_sim, "motor_right")
        self.platform = DifferentialDrive(motor_left, motor_right)

        # Init Robot arm, gripper limits [q_open, q_close] must be adjusted to gripper hardware!
        self.arm = VrepRoArmM1Sim(self._vrep_sim, gripper_limits=[-10,20]) #-1..20 is for custom 3 finger gripper sim model!

    @vrep
    def _vrep_init(self, scene):
        self._vrep_handle = RemoteAPIClient(verbose=False)
        self._vrep_sim = self._vrep_handle.require('sim')
        self._vrep_sim.stopSimulation(True)

        if scene is None:
            scene = "scene.ttt"

        # Convert scene_path to str if Path object
        if isinstance(self.scene_path, Path):
            self.scene_path = str(self.scene_path)

        self._vrep_sim.loadScene(self.scene_path)
        self._vrep_sim.startSimulation()

    def stop(self):
        print("TODO: Stop robot")
    
    
    
