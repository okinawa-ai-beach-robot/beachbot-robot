import math
from beachbot.manipulators.arm import Arm
import time
import serial
from threading import Thread
import json
import threading

from beachbot.config import logger

class RoArmM1(Arm):
    def __init__(
        self, rate_hz=20, serial_port="/dev/ttyUSB0", gripper_limits=None
    ) -> None:
        # Init superclass thread
        super().__init__(gripper_limits)
        self.interval = 1.0 / rate_hz
        self.is_connected = False
        self.device = None
        self._write_lock = threading.Lock()

        if serial_port is not None:
            self.is_connected = False
            try:
                self.device = serial.Serial(
                    serial_port, timeout=0, baudrate=115200
                )  # open serial port
                # self.device.open()
                self.write_dspl("Beachbot", "Python connected!")
                self.is_connected = self.device.isOpen()

            except Exception as e:
                print("error open serial port: " + str(e))
                raise e

        if self.is_connected:
            self.thread = Thread(target=self.run, daemon=True).start()

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
            self.device.flush()

    def write_dspl(self, line1=None, line2=None, line3=None, line4=None):   
        jsondata = {
                "T": 103
            }
        if line1: jsondata["L1"]=line1
        if line2: jsondata["L2"]=line2
        if line3: jsondata["L3"]=line3
        if line4: jsondata["L4"]=line4
        txdata = json.dumps(jsondata)
        self.write_io(txdata)

    def cleanup(self):
        self.close_io()

    def close_io(self):
        if self.device.isOpen():
            self.device.close()
        self.is_connected = False

    def refresh_robot_state(self):
        # request arm state
        self.write_io('{"T":5}')
        for strdata in self.device.readlines():
            if not strdata.startswith(b"{"):
                # Ignore debug information messages
                # Only interpred json data {....}, must start with '{'
                # Otherwise print message:
                print("Debug message from servo board:", strdata.decode())
                continue
            try:
                data = json.loads(strdata)
                if "T" not in data:
                    # robot status package recieved:
                    qs = [float(data.get("A" + str(num + 1), 0)) for num in range(5)]
                    taus = [float(data.get("T" + str(num + 1), 0)) for num in range(5)]
                    qs[4] = self.normalize_gripper_angle(qs[4])
                    qs_changed = any(
                        [math.fabs(a - b) > 0.1 for a, b in zip(qs, self.qs)]
                    )
                    with self._status_lock:
                        self.qs = qs
                        self.taus = taus
                    if qs_changed:
                        with self._joint_changed:
                            self._joint_changed.notify_all()
                else:
                    # response from send commands to motor board!
                    # print("response from servoboard:", data.decode())
                    # One can do something, check fi command was successful or so ... 
                    pass

            except Exception as ex:
                print("Read error:" + strdata.decode())
                print("Exception:", ex)
                self.close_io()

    def set_max_torque(self, jointnr, maxvalue):
        if jointnr > 0 and jointnr < 6 and maxvalue >= 0 and maxvalue <= 1000:
            data = json.dumps({"T": 100, f"Q{jointnr}": maxvalue})
            self.write_io(data)
        else:
            raise ValueError(
                "joint number range is 1 to 5 and value range is percent x10 (0-1000)"
            )

    def set_joint_targets(self, qs):
        self.qs_target = qs
        percent = self.denormalize_gripper_angle(qs[4])

        # TODO add simple bounds/in-range check!
        data = json.dumps(
            {
                "T": 1,
                "P1": qs[0],
                "P2": qs[1],
                "P3": qs[2],
                "P4": qs[3],
                "P5": percent,
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
        # if pos < 0:
        #     pos = 0
        # if pos > 1:
        #     pos = 1
        # jpos = self.gripper_open + (self.gripper_close - self.gripper_open) * pos
        qt = self.get_joint_angles()
        qt[-1] = pos
        # print("set gripper", jpos)
        self.set_joint_targets(qt)

    def set_joints_enabled(self, is_enabled):
        if is_enabled:
            self.write_io('{"T":9,"P1":8}\n')
        else:
            self.write_io('{"T":9,"P1":7}\n')


class RoArmM1_Custom3FingerGripper(RoArmM1):
    def __init__(self, rate_hz=20, serial_port="/dev/ttyUSB0", gripper_limits=[42, 60]):
        super().__init__(rate_hz, serial_port, gripper_limits)
