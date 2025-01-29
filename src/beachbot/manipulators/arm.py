from time import sleep, time
from pathlib import Path
import numpy as np
import math
from beachbot.config import config
import threading


class Arm:

    def __init__(self, gripper_limits=None) -> None:
        self.basepath = config.BEACHBOT_HOME

        # Robot arm definition (mechanical structure):
        self.LEN_A = 115.432  # Height offset of base
        self.LEN_B = (
            41.0513  # Translation (x) offset of base rotation joint (1st joint)
        )
        self.LEN_C = 168.8579  # Length of first arm link (joint 2 to 3)
        self.LEN_D = 127.9234  # Length of secon arm link (joint 3 to 4)

        self.LEN_E = 108.5357  # Length of gripper (from joint4 to grasping poit)
        self.LEN_F = 7.7076  # Z Offset of gripper

        self.LEN_G = (
            90.0  # Offset (degree) of last rotational joint (joint 4) of gripper
        )
        self.LEN_H = (
            -13.75
        )  # shift along y of 1st rotational axis (base, joint 1) of arm. Arm is mounted "off-center".

        # Factor (constant) for modulation of relative orintation of joint4 to desired gripper orientation
        self.rateTZ = 2

        # Home position in cartesian space:
        self.initPosX = self.LEN_B + self.LEN_D + self.LEN_E
        self.initPosY = self.LEN_H
        self.initPosZ = self.LEN_A + self.LEN_C - self.LEN_F
        self.initPosT = 90

        self.gripper_open = None
        self.gripper_close = None
        if gripper_limits is None:
            self.gripper_open = 180
            self.gripper_close = 260
        else:
            self.gripper_open = gripper_limits[0]
            self.gripper_close = gripper_limits[1]

        self.q_home = [
            180.0,
            -12,
            100.2832031,
            45.0,
            self.gripper_open,
        ]  # Joint angle home position

        self.qs = self.q_home
        self.qs_target = None

        
        self._status_lock = threading.Lock()
        self._joint_changed = threading.Condition()

        asset_path = Path(__file__).parent.parent / "assets"
        pickup_path = asset_path / "pickup.npz"
        toss_path = asset_path / "toss.npz"
        self.pickup_trajectory = Trajectory.from_file(pickup_path)
        self.toss_trajectory = Trajectory.from_file(toss_path)

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
        print(
            "rotate",
            p[0],
            angle_1 - 180,
        )
        p_plane = rotate((p[0], self.LEN_H), angle_1 - 180)
        print("rotate", p[0], angle_1 - 180, p_plane)
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

    def get_joint_angles(self):
        with self._status_lock:
            return self.qs.copy()

    def get_joint_torques(self):
        with self._status_lock:
            return self.taus.copy()

    def get_joint_state(self):
        res = (self.get_joint_angles(), self.get_joint_torques())
        return res

    def get_joint_targets(self):
        with self._status_lock:
            return self.qs_target.copy()

    def set_joint_targets(self):
        """
        Overridden by child classes as API differs greatly
        """
        pass

    def normalize_gripper_angle(self, angle):
        """
        Convert from physical gripper angles to normalized values [0..1].
        Gripper interface to accept values [0..1], but actual serial interface to
        servos accepts actual angle values. We limit these values to between
        gripper_open and gripper_close.
        """
        return (angle - self.gripper_open) / (self.gripper_close - self.gripper_open)

    def denormalize_gripper_angle(self, percentage):
        """
        Convert from normalized values [0..1] to physical gripper angles.
        Gripper interface to accept values [0..1], but actual serial interface to
        servos accepts actual angle values. We limit these values to between
        gripper_open and gripper_close.
        """
        if percentage < 0 or percentage > 1:
            raise ValueError("Angle must be between 0 and 1")
        return self.gripper_open + (self.gripper_close - self.gripper_open) * percentage

    def set_gripper(self, percent):
        """
        Set gripper from open 0, to closed 1
        """

    def close_gripper(self, timeout=1) -> bool:
        """
        Close gripper
        args:
            timeout: time to wait for gripper to close
        returns:
            True if gripper closed within timeout
            False if gripper not closed within timeout
        """
        self.set_gripper(1)

        # Wait for gripper to close
        timeout = time() + timeout
        while 1 - self.get_joint_angles()[4] > 0.01:
            sleep(0.1)
            if time() > timeout:
                return False

        return True

    def open_gripper(self, timeout=1) -> bool:
        """
        Open gripper
        args:
            timeout: time to wait for gripper to open
        returns:
            True if gripper opened within timeout
            False if gripper not opened within timeout
        """
        self.set_gripper(0)

        # Wait for gripper to open
        timeout = time() + timeout
        while self.get_joint_angles()[4] > 0.01:
            sleep(0.1)
            if time() > timeout:
                return False

        return True

    def set_joints_enabled(self, is_enabled):
        """
        Overridden by child classes as API differs greatly
        TODO @Jeffrey add what and how this is supposed to work
        as it is very unclear at the moment.
        """
        pass

    def wait_for_movement(self, timeout=None):
        with self._joint_changed:
            res = self._joint_changed.wait(timeout)
        return res

    def record_trajectory(
        self, resample_steps=-1, wait_time_max=10, max_record_steps=250, save_path=None
    ):
        self.set_joints_enabled(False)
        sleep(0.5)
        q, tau = self.get_joint_state()
        qs = [q]
        taus = [tau]
        ts = [0]
        ts_start = time()

        rec_steps = 0
        while self.wait_for_movement(wait_time_max) and rec_steps <= max_record_steps:
            # Robot moved, record new position
            q, tau = self.get_joint_state()
            qs.append(q)
            taus.append(tau)
            ts.append(time() - ts_start)
            rec_steps += 1

        qs = np.stack(qs)
        taus = np.stack(taus)

        if resample_steps > 2:
            qs_resample = signal.resample(qs, resample_steps)
            qs_resample[-1, :] = qs[-1, :]
            qs = qs_resample

            taus_resample = signal.resample(taus, resample_steps)
            taus_resample[-1, :] = taus[-1, :]
            taus = taus_resample

            ts_resample = signal.resample(ts, resample_steps)
            ts_resample[-1, :] = ts[-1, :]
            ts = ts_resample

        if save_path:
            np.savez(save_path, qs=qs, taus=taus, ts=ts)

        return qs, taus, ts

    def replay_trajectory(self, qs, ts=None, freq=20, speed_factor=1):
        """
        Replay recorded trajectory
        args:
            qs: array of joint angles
            ts: array of timestamps
            freq: playback frequency (Optional) used if ts is None
            speed_factor: scaling factor to divide ts by in order to speed up playback
        """
        # Set initial position and wait
        self.set_joint_targets(qs[0])
        self.wait_joint_target_arrival()
        ts = ts / speed_factor
        ts_start = time()

        # Index starts from 1 to allow ts difference calc
        # Ends two from the end to allow any speed_factor while waiting
        # for final position before opening/closing gripper
        for t in range(1, qs.shape[0] - 2):
            self.set_joint_targets(qs[t])
            sleep_time = 0
            ts_now = time()
            if ts is None:
                # fixed replay frequency
                sleep_time = (1.0 / freq) - (ts_now - ts_start)
                ts_start = ts_now
            else:
                # wait for timestamp
                sleep_time = ts[t] - ts[t - 1]

            if sleep_time > 0:
                sleep(sleep_time)
        self.set_joint_targets(qs[-2])
        self.wait_joint_target_arrival()
        self.set_joint_targets(qs[-1])

    def go_home(self):
        self.set_joint_targets(self.q_home)
        sleep(1)

    def is_ready(self):
        """Robot arm connected and ready for operation"""
        return self.is_connected

    def get_position(self):
        """Returns the current position of the gripper"""
        pass

    # Helper function for inverse kinematic calculations,
    # as provided by waveshare (rewritten in python)
    def simpleLinkageIK(self, LA, LB, aIn, bIn):
        if bIn == 0:
            psi = (
                math.acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn))
                * 180
                / math.pi
            )
            alpha = 90 - psi
            omega = (
                math.acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB))
                * 180
                / math.pi
            )
            beta = psi + omega
        else:
            L2C = aIn * aIn + bIn * bIn
            LC = math.sqrt(L2C)
            lamb = math.atan(bIn / aIn) * 180 / math.pi
            psi = math.acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) * 180 / math.pi
            alpha = 90 - lamb - psi
            omega = math.acos((LB * LB + L2C - LA * LA) / (2 * LC * LB)) * 180 / math.pi
            beta = psi + omega

        delta = 90 - alpha - beta

        if math.isnan(alpha) or math.isnan(beta) or math.isnan(delta):
            raise Exception("NAN")

        angle_2 = alpha
        angle_3 = beta
        angle_IKE = delta
        return angle_2, angle_3, angle_IKE

    def EoAT_IK(self, angleInput):
        if angleInput == 90:
            betaGenOut = angleInput - self.LEN_G
            betaRad = betaGenOut * math.pi / 180
            angleRad = angleInput * math.pi / 180
            aGenOut = self.LEN_E
            bGenOut = self.LEN_F
        elif angleInput < 90:
            betaGenOut = 90 - angleInput
            betaRad = betaGenOut * math.pi / 180
            angleRad = angleInput * math.pi / 180
            aGenOut = math.cos(angleRad) * self.LEN_F + math.cos(betaRad) * self.LEN_E
            bGenOut = math.sin(angleRad) * self.LEN_F - math.sin(betaRad) * self.LEN_E
            betaGenOut = -betaGenOut
        elif angleInput > 90:
            betaGenOut = self.LEN_G - (180 - angleInput)
            betaRad = betaGenOut * math.pi / 180
            angleRad = angleInput * math.pi / 180
            aGenOut = (
                -math.cos(math.pi - angleRad) * self.LEN_F
                + math.cos(betaRad) * self.LEN_E
            )
            bGenOut = (
                math.sin(math.pi - angleRad) * self.LEN_F
                + math.sin(betaRad) * self.LEN_E
            )

        if math.isnan(betaGenOut):
            raise Exception("NAN")

        angle_EoAT = betaGenOut
        len_a = aGenOut
        len_b = bGenOut
        return angle_EoAT, len_a, len_b

    def wigglePlaneIK(self, LA, aIn, bIn):
        bIn = -bIn
        if bIn > 0:
            L2C = aIn * aIn + bIn * bIn
            LC = math.sqrt(L2C)
            lamb = math.atan(aIn / bIn) * 180 / math.pi
            psi = math.acos(LA / LC) * 180 / math.pi
            LB = math.sqrt(L2C - LA * LA)
            alpha = psi + lamb - 90
        elif bIn == 0:
            alpha = 90 + math.asin(LA / aIn) * 180 / math.pi
            L2C = aIn * aIn + bIn * bIn
            LB = math.sqrt(L2C)
        elif bIn < 0:
            bIn = -bIn
            L2C = aIn * aIn + bIn * bIn
            LC = math.sqrt(L2C)
            lamb = math.atan(aIn / bIn) * 180 / math.pi
            psi = math.acos(LA / LC) * 180 / math.pi
            LB = math.sqrt(L2C - LA * LA)
            alpha = 90 - lamb + psi

        if math.isnan(alpha):
            raise Exception("NAN")

        angle_1 = alpha + 90
        len_totalXY = LB - self.LEN_B
        return angle_1, len_totalXY

    def move_to(self, targetpos):
        """Move gripper to a certain target position"""
        pass

    def cleanup(self):
        pass

    def wait_joint_target_arrival(
        self, max_distance=1.0, timeout=10, polling_interval=0.1
    ):
        """
        Wait until actual joint angles are within the target range or timeout.
        Parameters:
        - max_distance: float, maximum allowable distance to the target.
        - timeout: float, maximum time to wait in seconds.
        - polling_interval: float, time between position checks.
        Returns:
        - bool: True if the target was reached, False if timed out.
        """

        t_start = time()
        while True:
            qs = self.get_joint_angles()
            qs_target = self.get_joint_targets()
            dist = np.linalg.norm(qs - qs_target)
            if np.linalg.norm(qs - qs_target) <= max_distance:
                return True
            if time() - t_start > timeout:
                return False
            print("Distance to target: ", dist)
            print("Current joint angles: ", qs)
            print("Target joint angles: ", qs_target)
            sleep(polling_interval)

    def pickup(self, speed_factor=20):
        self.replay_trajectory(
            self.pickup_trajectory.qs, self.pickup_trajectory.ts, speed_factor=speed_factor
        )

    def toss(self, speed_factor=20):
        self.replay_trajectory(
            self.toss_trajectory.qs, self.toss_trajectory.ts, speed_factor=speed_factor
        )


class Trajectory:
    def __init__(self, ts=None, qs=None, taus=None):
        self.ts = ts
        self.qs = qs
        self.taus = taus

    @classmethod
    def from_file(cls, trajectory_path: str):
        """Class method to create a Trajectory instance from a file."""
        data = np.load(trajectory_path)
        return cls(ts=data["ts"], qs=data["qs"], taus=data["taus"])


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
