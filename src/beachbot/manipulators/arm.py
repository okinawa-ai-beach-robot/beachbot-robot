from time import sleep, time
import numpy as np
import math
try:
    from importlib.resources import files  # Python 3.9+
except ImportError:
    from importlib_resources import files  # Backport for Python 3.8


class Arm:

    def fkin(self, qs):
        """
        x,y,z = fkin([angle_1, angle_2, angle_3, angle_4])
        Calculate gripper position and rotation based on given joint angles

        Code reconstructed from given inverse kinamtic routines provided by waveshare.
        """
        angle_1, angle_2, angle_3, angle_4 = qs

        # First, calculate position in x/z plane (joint 2 - joint 4)
        # Forward kinematic is calculated from tip of the robot backward to base!
        # Start position:
        p = (0, 0)
        # gGripper offset translation:
        p = translate(p, (self.LEN_E, -self.LEN_F))
        # Gripper rotation (joint 4):
        p = rotate(p, -angle_4)
        # Translation from joint 4 to joint 3:
        p = translate(p, (self.LEN_D, 0))
        # Joint 3 rotation:
        p = rotate(p, -angle_3)
        # Translation from joint 3 to joint 2:
        p = translate(p, (self.LEN_C, 0))
        # Joint 2 Rotation
        p = rotate(p, 90 - angle_2)
        # Offset translation from joint 2 to base rotation
        p = translate(p, (self.LEN_B, self.LEN_A))

        # Rotate robot arm in x/y plane (joint 1):
        print("rotate", p[0],  angle_1 - 180, )
        p_plane = rotate((p[0], self.LEN_H), angle_1 - 180)
        print("rotate", p[0],  angle_1 - 180, p_plane)
        # Combine X/Z and X/Y plane caculation for final result:
        return (p_plane[0], p_plane[1], p[1])

    def inv_kin(self, position, eoat):
        """
        angle_1, angle_2, angle_3, angle_4 = inv_kin([InputX, InputY, InputZ], eoat)
        Calculate required joint angles to reach a certain position/posture.
        position: cartesian position
        EOAT: desired End of Arm Tooling, rotation angle of gripper

        Code provided by waveshare, rewritten in python
        """
        InputX, InputY, InputZ = position
        InputTheta = eoat

        # Rotation of the gripper (relative to arm) is defined as a constant fraction of the desired rotation:
        # (ambiguity resolution)
        InputTheta = eoat / self.rateTZ
        # Joint 1 rotates the robot arm in the X/Y plane, calculate angles based on desired target X/Y position:
        angle_1, len_totalXY = self.wigglePlaneIK(self.LEN_H, InputX, InputY)
        # Calculate offset in XZ plane from last joint (joint 4) to gripper
        angle_EoAT, len_a, len_b = self.EoAT_IK(InputTheta)
        # Calculate required angles of arm joints (2,3) such that the target with desired rotation of the gripper cna be reached:
        angle_2, angle_3, angle_IKE = self.simpleLinkageIK(
            self.LEN_C, self.LEN_D, (len_totalXY - len_a), (InputZ - self.LEN_A + len_b)
        )
        # Estimate required joint angle of the gripper rotation (joint 4):
        angle_4 = angle_IKE + angle_EoAT

        return angle_1, angle_2, angle_3, angle_4




class Trajectory:
    def __init__(self, ts=None, qs=None, taus=None):
        self.ts = ts
        self.qs = qs
        self.taus = taus

    @classmethod
    def from_file(cls, trajectory_path: str):
        """Class method to create a Trajectory instance from a file."""
        data = np.load(trajectory_path)
        return cls(ts=data['ts'], qs=data['qs'], taus=data['taus'])


def rotate(point, angle, origin=(0, 0)):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle must be given in degrees.
    """
    ox, oy = origin
    px, py = point

    angle = angle * math.pi / 180

    print("rotimpl", angle, math.cos(angle), math.sin(angle))

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    print(oy, math.sin(angle) * (px - ox), math.cos(angle) * (py - oy))
    return qx, qy


def translate(point, offset):
    """
    Translate point by offset.
    """
    return point[0] + offset[0], point[1] + offset[1]


def load_trajectory(trajectory: str):
    trajectory_path = files('beachbot.assets').joinpath(trajectory)
    data = np.load(trajectory_path)
    ts = data["ts"]
    qs = data["qs"]
    taus = data["taus"]
    return ts, qs, taus


def load_default_trajectories():
    global pickup_trajectory, toss_trajectory
    pickup_trajectory = Trajectory.from_file('beachbot.assets/pickup.npz')
    toss_trajectory = Trajectory.from_file('beachbot.assets/toss.npz')


def replay_trajectory(self,
                      qs: np.ndarray,
                      ts: np.ndarray = None,
                      timestep_length: float = 0.01):
    """
    Replay a trajectory.

    Parameters:
    - qs: numpy.ndarray, shape (n, m), where n is the number of time steps and m is the number of joints.
    - ts: numpy.ndarray, shape (n,), optional, timestamps for each time step.
    - timestep_length: float, optional, length of each time step in seconds. Will default to 0.01 if ts is not provided.
    """

    # Set initial position as initial timestamp should be around 0
    self.set_joint_targets(qs[0])
    self.wait_target_arrival()
    sleep_time = 0

    if ts is None:
        for t in range(1, qs.shape[0]):
            self.set_joint_targets(qs[t])
            sleep_time = timestep_length
            sleep(sleep_time)
    else:
        for t in range(1, qs.shape[0]):
            self.set_joint_targets(qs[t])
            sleep_time = ts[t] - ts[t-1]
            sleep(sleep_time)


def wait_target_arrival(self, max_distance=0.02, timeout=10, polling_interval=0.1):
    """
    Wait until the gripper position is within the target range or timeout.

    Parameters:
    - max_distance: float, maximum allowable distance to the target.
    - timeout: float, maximum time to wait in seconds.
    - polling_interval: float, time between position checks.

    Returns:
    - bool: True if the target was reached, False if timed out.
    """

    t_start = time()
    while True:
        pos = self.get_gripper_pos()
        target = self.get_gripper_target()
        if np.linalg.norm(pos - target) <= max_distance:
            return True
        if time() - t_start > timeout:
            return False
        sleep(polling_interval)


def pickup(self):
    self.replay_trajectory(pickup_trajectory)


def toss(self):
    self.replay_trajectory(toss_trajectory)


pickup_trajectory: Trajectory = None
toss_trajectory: Trajectory = None
load_default_trajectories()
