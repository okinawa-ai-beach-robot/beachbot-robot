import os
import math

import numpy as np

from ..utils.vrepsimulation import vrep
from ..config import config

try:
    from scipy import signal
except:
    print("scipy requites dorgraded numpy version... do a pip install numpy==1.22")

import threading, time, io
import json

try:
    import serial

    _ser_intf = serial.Serial
except:
    print(
        "Could not load pyserial, please make sure -pyserial- and not package -serial- is installed!"
    )


def rotate(point, angle, origin=(0, 0)):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle must be given in degrees.
    """
    ox, oy = origin
    px, py = point

    angle = angle * math.pi / 180

    print("rotimpl", angle, math.cos(angle), math.sin(angle) )

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    print(oy , math.sin(angle) * (px - ox) , math.cos(angle) * (py - oy))
    return qx, qy


def translate(point, offset):
    """
    Translate point by offset.

    """
    return point[0] + offset[0], point[1] + offset[1]


class VrepRoArmM1Sim():
    def __init__(self, vrep_sim, gripper_limits=None) -> None:
        self.vrep_sim = vrep_sim
     
        self.is_connected = False
        #self._status_lock = threading.Lock()

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

        # Joint offsets to transform robot joints to simulator joints:
        self.q_zero = [
            180.0,
            45.0,
            0.0,
            0.0, 
            -self.gripper_open,
        ]  # Joint angle home position

        self.q_zero_fac = [
            -1.0,
            -1.0,
            -1.0,
            -1.0
        ]  # Joint angle home position


        self.qs = self.q_home

        self.vrep_jointnames_arm=['base_to_L1', 'L1_to_L2', 'L2_to_L3', 'L3_to_L4', ]
        #self.vrep_jointnames_gripper=['L4_to_L5_1_A', 'L4_to_L5_1_B' ]
        self.vrep_jointnames_gripper=['finger_joint_1', 'finger_joint_2', 'finger_joint_3' ]

        # Important, mark interacitons with sim via function deccorator '@vrep' to execute them in the same thread that established the simulator connection
        self.init_sim()



    @vrep
    def init_sim(self):
        self.vrep_jointids_arm = [self.vrep_sim.getObject("/"+jn) for jn in self.vrep_jointnames_arm]
        self.vrep_jointids_gripper = [self.vrep_sim.getObject("/"+jn) for jn in self.vrep_jointnames_gripper]

        self.vrep_base_id = self.vrep_sim.getObject("/arm_mount")
        self.vrep_gripper_id = self.vrep_sim.getObject("/Tip")
        # todo optional, wrap with try catch, or does it return None? ->
        try:
            self.vrep_path_script = self.vrep_sim.getObject("./tipctrl")
        except:
            self.vrep_path_script=None
        self.vrep_robot_script = self.vrep_sim.getObject("./robotctrl")

    @vrep
    def set_target_path_pos(self, percent=0.5, offset=[0,0,0]):
        if self.vrep_path_script is not None:
            self.vrep_sim.callScriptFunction("setPathPosition",self.vrep_path_script,percent, offset)


    def set_cart_pos(self, pos_target, tool_angle):
        # target relative to home position:
        qs = self.inv_kin([t+o for  t,o in zip(pos_target, self.cart_home)], tool_angle+self.cart_gripper_angle_home)

        # Related hardware control code:
        # jsonPosCtrl[0] = (int)(angleGenOut(STDirection[0]*jsonCmdReceive["P1"].as<float>()) + 0.5);
        # jsonPosCtrl[1] = (int)(angleGenOut(STDirection[1]*jsonCmdReceive["P2"].as<float>()+15)*3 + 0.5);
        # jsonPosCtrl[2] = (int)(angleGenOut(STDirection[2]*jsonCmdReceive["P3"].as<float>()+180) + 0.5);
        # jsonPosCtrl[3] = (int)(angleGenOut(STDirection[3]*jsonCmdReceive["P4"].as<float>()+180) + 0.5);
        # jsonPosCtrl[4] = (int)(angleGenOut(STDirection[4]*jsonCmdReceive["P5"].as<float>()) + 0.5);
        
        #   STPos[0] = (int)(angleGenOut(STDirection[0]*InfoBuffer[angle_1]) + 0.5);
        #   STPos[1] = (int)(angleGenOut(STDirection[1]*InfoBuffer[angle_2]+15)*3 + 0.5);
        #   STPos[2] = (int)(angleGenOut(STDirection[2]*InfoBuffer[angle_3]+180)  + 0.5);
        #   STPos[3] = (int)(angleGenOut(STDirection[3]*InfoBuffer[angle_4]+180)  + 0.5);
        #   //st.SyncWritePosEx(STID, 4, STPos, STSpd, STAc);



        test2 = self.fkin(qs)
        print([t+o for  t,o in zip(pos_target, self.cart_home)], qs, test2)

        qs = list(qs)
        #qs[1]*=-1
        qs[3]*=-1




        self.set_joint_targets(qs,do_offsetcompensation=True)



    def _get_gripper(self):
        q,t = self.vrep_sim.callScriptFunction("get_gripper_percent",self.vrep_robot_script)
        return q,t
    def _set_gripper(self, val_percent):
        self.vrep_sim.callScriptFunction("set_gripper_percent",self.vrep_robot_script, val_percent)

      
    @vrep
    def get_joint_angles(self, do_offsetcompensation=True):
        res = [self.vrep_sim.getJointPosition(jid)*180/math.pi for jid in self.vrep_jointids_arm]
        q,t =  self._get_gripper()
        res += [q]
        if do_offsetcompensation:
            for i in range(4):
                res[i] = self.q_zero_fac[i]*(res[i] - self.q_zero[i])
        return res

    @vrep
    def get_joint_torques(self):
        res = [self.vrep_sim.getJointForce(jid) for jid in self.vrep_jointids_arm]
        q,t =  self._get_gripper()
        res += [t]
        return res

    
    def get_joint_state(self):
        #with self._status_lock:
        res = (self.get_joint_angles(), self.get_joint_torques())
        return res


    @vrep
    def set_joint_targets(self, qs, do_offsetcompensation=True):
        for i in range(4):
            if do_offsetcompensation:
                qt = self.q_zero[i] + self.q_zero_fac[i]*qs[i]
            else:
                qt = qs[i]
            self.vrep_sim.setJointTargetPosition(self.vrep_jointids_arm[i], qt*math.pi/180)
        if len(qs)>4:
            self._set_gripper(qs[4])

    # def _set_gripper(self, pos):
    #     if pos < 0:
    #         pos = 0
    #     if pos > 1:
    #         pos = 1
    #     jpos = self.gripper_open + (self.gripper_close - self.gripper_open) * pos
    #     for finger_jid in self.vrep_jointids_gripper:
    #         q_gripper=jpos
    #         self.vrep_sim.setJointTargetPosition(finger_jid, q_gripper)


    @vrep
    def set_gripper(self, pos):
        self._set_gripper(pos)

    @vrep
    def set_joints_enabled(self, is_enabled):
        for jid in self.vrep_jointids_arm:
            if is_enabled:
                self.vrep_sim.setJointMode(jid, self.vrep_sim.jointdynctrl_position, 0)
            else:
                self.vrep_sim.setJointMode(jid, self.vrep_sim.jointdynctrl_free, 0)

    def wait_for_movement(self, timeout=None):
        qs_old = self.get_joint_angles()
        t_start = time.time()
        t_now=t_start
        while timeout==None or t_now-t_start<timeout:
            time.sleep(0.1)
            qs_new = self.get_joint_angles()
            qs_changed = any(
                    [math.fabs(a - b) > 0.1 for a, b in zip(qs_old, qs_new)]
            )
            if qs_changed:
                return True
        return False

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
    
    def cleanup(self):
        pass

    @vrep
    def get_gripper_pos(self):
        pos = self.vrep_sim.getObjectPosition(self.vrep_gripper_id, self.vrep_base_id)
        return [pos[1]*1000, -pos[0]*1000, pos[2]*1000]

    def replay_trajectory(self, qs, ts=None, freq=20, gripper_overwrite=None):
        print("replay")
        #self.set_joints_enabled(True)
        time.sleep(0.5)
        ts_start = time.time()

        for t in range(qs.shape[0]):
            print("replay", t)
            if gripper_overwrite is not None:
                qs[t][-1]=gripper_overwrite
            self.set_joint_targets(qs[t])
            wtime = 0
            ts_now = time.time()
            if ts == None:
                # fixed replay frequency
                wtime = (1.0 / freq) - (ts_now - ts_start)
                ts_start = ts_now
            else:
                # wait for timestamp
                wtime = ts[t] - (ts_now - ts_start)

            if wtime > 0:
                time.sleep(wtime)
        print("end")

    def go_home(self):
        self.set_joint_targets(self.q_home)
        time.sleep(1)

    def go_zero(self):
        self.set_joint_targets([0,0,0,0]+[self.q_zero[-1]])
        time.sleep(1)

    def go_calib(self):
        #self.set_joint_targets(self.q_zero)
        self.set_joint_targets([180,75, 0, 0]+[self.q_zero[-1]])
        time.sleep(1)

    def test_movement(self):
        pathfile = os.path.dirname(__file__)
        # Load the .npz file
        data = np.load(str(pathfile) + "/../../../arm/pickup_path.npz")
        #data2 = np.load(str(pathfile) + "/../../../arm/toss_path.npz")

        # Access the arrays by their keys
        qs_grab = data["qs"]
        taus_grab = data["taus"]
        ts_grab = data["ts"].tolist()
        print(ts_grab)

        # data = data2

        # # Access the arrays by their keys
        # qs_toss = data["qs"]
        # taus_toss = data["taus"]
        # ts_toss = data["ts"].tolist()

    
        #self.refresh_robot_state()
        #self.go_home()
        self.replay_trajectory(qs_grab, ts_grab)
        time.sleep(1)
        # close gripper
        #self.set_gripper(0.8)
        #time.sleep(1)
        # self.replay_trajectory(qs_toss, ts_toss)
        # time.sleep(1)

        # open gripper
        #self.set_gripper(0.0)
        #time.sleep(1)
        #self.go_home()
        print("Done!")


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
        print("rotate",p[0],  angle_1 - 180, )
        p_plane = rotate((p[0], self.LEN_H), angle_1 - 180)
        print("rotate",p[0],  angle_1 - 180, p_plane)
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


class VrepRoArmM1SimCustom3Finger(VrepRoArmM1Sim):
    def __init__(self, vrep_sim, gripper_limits=[-10,20]):
        super().__init__(vrep_sim, gripper_limits)