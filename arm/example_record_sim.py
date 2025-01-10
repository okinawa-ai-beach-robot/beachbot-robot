import time
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm


def pickup():
    pathpospercent = 0.25
    increment = -0.05
    q = simarm.get_joint_state()[0]
    tau = simarm.get_joint_state()[1]

    while pathpospercent > 0.0:
        time.sleep(0.1)
        q, tau = simarm.get_joint_state()
        pathpospercent += increment
        simarm.set_target_path_pos(percent=pathpospercent, offset=[0, 0, 0])

    # How to match current state with target, else it will close prior to arriving at ground 
    time.sleep(2)
    q[4] = 1
    simarm.set_joint_targets(q)


def toss():
    pathpospercent = 0.0
    increment = 0.05

    while pathpospercent < 1.0:
        time.sleep(0.1)
        q, tau = simarm.get_joint_state()
        pathpospercent += increment
        simarm.set_target_path_pos(percent=pathpospercent, offset=[0, 0, 0])

    # How to match current state with target, else it will close prior to arriving at ground 
    time.sleep(3)
    q[4] = 0
    simarm.set_joint_targets(q)


simarm.go_home()
pickup()
toss()
