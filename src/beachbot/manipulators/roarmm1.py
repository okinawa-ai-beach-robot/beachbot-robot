import math
import numpy as np
from beachbot.config import config
import threading
import time
import json
from scipy import signal
import serial


class RoArmM1(threading.Thread):
    def __init__(self, rate_hz=20, serial_port="/dev/ttyUSB0", gripper_limits=None) -> None:
        # Init superclass thread
        super().__init__()
        # do not block on exit:
        self.daemon = True
        self.interval = 1.0 / rate_hz
        self.is_connected = False
        self.device = None
        if serial_port is not None:
            self.is_connected = False
            try:
                self.device = serial.Serial(
                    serial_port, timeout=0, baudrate=115200
                )  # open serial port
                # self.device.open()
                self.is_connected = self.device.isOpen()
            except Exception as e:
                print("error open serial port: " + str(e))

        self._write_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._joint_changed = threading.Condition()
        self._joint_targets = None

        if self.is_connected:
            super().start()

    def run(self):
        while self.is_connected:
            functime = time.time()
            self.refresh_robot_state()
            functime = time.time() - functime

            if functime < self.interval:
                time.sleep(self.interval - functime)
            else:
                raise Exception(
                    "Error: Out of time! ("
                    + str(functime)
                    + ","
                    + str(self.interval)
                    + ")"
                )

    def write_io(self, data):
        with self._write_lock:
            self.device.write(data.encode())

    def cleanup(self):
        self.close_io()

    def close_io(self):
        if self.device.isOpen():
            self.device.close()
        self.is_connected = False

    def refresh_robot_state(self):
        # request arm state
        self.write_io('{"T":5}\n')
        for strdata in self.device.readlines():
            if not strdata.startswith(b"{"):
                # Ignore information messages
                # Only interpred json data {....}
                print("msg:", strdata.decode())
                continue
            try:
                data = json.loads(strdata)
                if "T" not in data:
                    # robot status package recieved:
                    qs = [float(data.get("A" + str(num + 1), 0)) for num in range(5)]
                    taus = [float(data.get("T" + str(num + 1), 0)) for num in range(5)]
                    # Convert gripper joint angle into range 0..1
                    qs[4] = (qs[4] - self.gripper_open) / (self.gripper_close - self.gripper_open)
                    qs_changed = any(
                        [math.fabs(a - b) > 0.1 for a, b in zip(qs, self.qs)]
                    )
                    with self._status_lock:
                        self.qs = qs
                        self.taus = taus
                    if qs_changed:
                        with self._joint_changed:
                            self._joint_changed.notify_all()
            except Exception as ex:
                print("Read error:" + strdata.decode())
                print("Exception:", ex)
                self.close_io()

    def get_joint_angles(self):
        with self._status_lock:
            res = self.qs.copy()
        return res
    
    def set_max_torque(self, jointnr, maxvalue):
        if jointnr>0 and jointnr<6 and maxvalue>=0 and maxvalue<=1000:
            data = json.dumps(
                {
                    "T": 100,
                    f"Q{jointnr}": maxvalue
                }
            )
            self.write_io(data)
        else:
            raise ValueError("joint number range is 1 to 5 and value range is percent x10 (0-1000)")

    def get_joint_torques(self):
        with self._status_lock:
            res = self.taus.copy()
        return res

    def get_joint_state(self):
        with self._status_lock:
            res = (self.qs.copy(), self.taus.copy())
        return res

    def set_joint_targets(self, qs):
        # TODO add simple bounds/in-range check!
        data = json.dumps(
            {
                "T": 1,
                "P1": qs[0],
                "P2": qs[1],
                "P3": qs[2],
                "P4": qs[3],
                "P5": qs[4],
                "S1": 400,
                "S2": 1200,
                "S3": 400,
                "S4": 400,
                "S5": 400,
                "A1": 60,
                "A2": 60,
                "A3": 60,
                "A4": 60,
                "A5": 60,
            }
        )
        self.write_io(data)

    def set_gripper(self, pos):
        if pos < 0:
            pos = 0
        if pos > 1:
            pos = 1
        jpos = self.gripper_open + (self.gripper_close - self.gripper_open) * pos
        qt = self.get_joint_angles()
        qt[-1] = jpos
        print("set gripper", jpos)
        self.set_joint_targets(qt)

    def set_joints_enabled(self, is_enabled):
        if is_enabled:
            self.write_io('{"T":9,"P1":8}\n')
        else:
            self.write_io('{"T":9,"P1":7}\n')

    def wait_for_movement(self, timeout=None):
        with self._joint_changed:
            res = self._joint_changed.wait(timeout)
        return res

    def record_trajectory(
        self, resample_steps=-1, wait_time_max=10, max_record_steps=250, save_path=None
    ):
        self.set_joints_enabled(False)
        time.sleep(0.5)
        q, tau = self.get_joint_state()
        qs = [q]
        taus = [tau]
        ts = [0]
        ts_start = time.time()

        rec_steps = 0
        while self.wait_for_movement(wait_time_max) and rec_steps <= max_record_steps:
            # Robot moved, record new position
            q, tau = self.get_joint_state()
            qs.append(q)
            taus.append(tau)
            ts.append(time.time() - ts_start)
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

    def replay_trajectory(self, qs, ts=None, freq=20, gripper_overwrite=None):
        self.set_joints_enabled(True)
        time.sleep(0.5)
        ts_start = time.time()

        for t in range(qs.shape[0]):
            if gripper_overwrite is not None:
                qs[t][-1]=gripper_overwrite
            self.set_joint_targets(qs[t])
            wtime = 0
            ts_now = time.time()
            if ts is None:
                # fixed replay frequency
                wtime = (1.0 / freq) - (ts_now - ts_start)
                ts_start = ts_now
            else:
                # wait for timestamp
                wtime = ts[t] - (ts_now - ts_start)

            if wtime > 0:
                time.sleep(wtime)

    def go_home(self):
        self.set_joint_targets(self.q_home)
        time.sleep(1)

    def is_ready(self):
        """Robot arm connected and ready for operation"""
        return self.is_connected

    def move_to(self, targetpos):
        """Move gripper to a certain target position"""
        pass

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


class RoArmM1_Custom3FingerGripper(RoArmM1):
    def __init__(self, rate_hz=20, serial_port="/dev/ttyUSB0", gripper_limits=[42,60]):
        super().__init__(rate_hz, serial_port, gripper_limits)
