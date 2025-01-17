import os
import math
import numpy as np
from beachbot.utils.vrepsimulation import vrep
from beachbot.config import config
import time
from scipy import signal
from beachbot.manipulators.arm import Arm


class VrepRoArmM1Sim(Arm):
    def __init__(self, vrep_sim, gripper_limits=None) -> None:
        super().__init__(gripper_limits)
        self.vrep_sim = vrep_sim
        self.is_connected = False
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

        self.vrep_jointnames_arm = ['base_to_L1', 'L1_to_L2', 'L2_to_L3', 'L3_to_L4', ]
        self.vrep_jointnames_gripper = ['finger_joint_1', 'finger_joint_2', 'finger_joint_3']

        # Important, mark interacitons with sim via function deccorator '@vrep' to execute them in the same thread that established the simulator connection
        self.init_sim()

    @vrep
    def init_sim(self):
        self.vrep_jointids_arm = [self.vrep_sim.getObject("/"+jn) for jn in self.vrep_jointnames_arm]
        self.vrep_jointids_gripper = [self.vrep_sim.getObject("/"+jn) for jn in self.vrep_jointnames_gripper]

        self.vrep_base_id = self.vrep_sim.getObject("/arm_mount")
        self.vrep_gripper_id = self.vrep_sim.getObject("/Tip")
        try:
            self.vrep_gripper_target_id = self.vrep_sim.getObject("/TipTarget")
        except:
            self.vrep_gripper_target_id = None
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
        q, t = self._get_gripper()
        res += [q]
        if do_offsetcompensation:
            for i in range(4):
                res[i] = self.q_zero_fac[i]*(res[i] - self.q_zero[i])
        return res

    @vrep
    def get_joint_torques(self):
        res = [self.vrep_sim.getJointForce(jid) for jid in self.vrep_jointids_arm]
        q, t = self._get_gripper()
        res += [t]
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

    @vrep
    def get_gripper_pos(self):
        """Return current gripper position in cartesian space (x,y,z)
        returns
        (x,y,z) gripper position or
        """
        pos = self.vrep_sim.getObjectPosition(self.vrep_gripper_id)
        return np.array(pos)

    @vrep
    def get_gripper_target(self):
        """Return current gripper target position in cartesian space (x,y,z)
        returns
        (x,y,z,x_target,y_target,z_target) if inverse kinemtaic target is available
        """
        if self.vrep_gripper_target_id:
            target = self.vrep_sim.getObjectPosition(self.vrep_gripper_target_id)
            return np.array(target)
        else:
            return None

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

class VrepRoArmM1SimCustom3Finger(VrepRoArmM1Sim):
    def __init__(self, vrep_sim, gripper_limits=[-10,20]):
        super().__init__(vrep_sim, gripper_limits)
