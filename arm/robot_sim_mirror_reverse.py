import time
import beachbot
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from beachbot.manipulators.roarmm1 import RoArmM1_Custom3FingerGripper

# confiog is 180 75 10 0 ...
# 2. joint readings -15 ... 105...-15.... cyclic!


arm1 = RoArmM1_Custom3FingerGripper()
arm1.go_home()
#arm1.set_max_torque(5, 0)

time.sleep(1.0)

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt") # scene3_invkin.ttt
simarm1 = robot.arm

arm1.go_home()
arm1.set_joints_enabled(True)

# time.sleep(2.0)
counter =0
pathpos=0.5
pathposstep=0.01
try:
    while True:
        time.sleep(0.1)
        counter +=1

        q,tau = simarm1.get_joint_state()
        arm1.set_joint_targets(q)

        pathpos += pathposstep
        if pathpos>1.0:
            pathposstep = pathposstep*-1
            pathpos=1
        elif pathpos<0:
            pathposstep = pathposstep*-1
            pathpos=0

        simarm1.set_target_path_pos(percent=pathpos, offset=[0,0,0.05]) #5 cm higher, otherwise long gripper collides with gorund


        if counter ==10:
            print("Current joint angles:\t", q)
            print("Current tourques:\t", tau)
            counter = 0
except KeyboardInterrupt:
    print('Bye Bye!')

