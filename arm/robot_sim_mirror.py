import time
import beachbot
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from beachbot.manipulators.roarmm1 import RoArmM1_Custom3FingerGripper


# confiog is 180 75 10 0 ...
# 2. joint readings -15 ... 105...-15.... cyclic!

arm1 = RoArmM1_Custom3FingerGripper()

robot = VrepRobotSimV1(scene="roarm_m1_locomotion_3finger.ttt")
simarm1 = robot.arm


arm1.set_joints_enabled(False)

counter =0
try:
    while True:
        time.sleep(0.1)
        counter +=1
        q,tau = arm1.get_joint_state()
        simarm1.set_joint_targets(q)
        simread = simarm1.get_joint_angles()

        if counter ==10:
            print("Current joint angles:\t", q)
            print("Current joint read from sim:\t", simread)
            print("Current tourques:\t", tau)
            counter = 0
except KeyboardInterrupt:
    print('Bye Bye!')

