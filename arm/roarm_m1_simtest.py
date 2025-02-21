import time
import beachbot
from beachbot.robot.robotinterface import RobotInterface
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
import math

epsilon = 1e-5

robot = VrepRobotSimV1("roarm_m1_locomotion_3finger.ttt")

arm = robot.arm


qs_poses = []

qs_poses.append(
    [
        (arm.LEN_B + arm.LEN_C + arm.LEN_D + arm.LEN_E),
        arm.LEN_H,
        arm.LEN_A - arm.LEN_F,
        180,
    ]
)
qs_poses.append(
    [
        (arm.LEN_B + arm.LEN_C + arm.LEN_D + arm.LEN_E) / 2,
        arm.LEN_H,
        arm.LEN_A - arm.LEN_F,
        180,
    ]
)
qs_poses.append(
    [
        (arm.LEN_B + arm.LEN_C + arm.LEN_D + arm.LEN_E) / 2,
        arm.LEN_H,
        arm.LEN_A - arm.LEN_F + 50,
        150,
    ]
)
qs_poses.append(
    [
        (arm.LEN_B + arm.LEN_C + arm.LEN_D + arm.LEN_E) / 2,
        arm.LEN_H,
        arm.LEN_A - arm.LEN_F,
        170,
    ]
)

for pos in []: #qs_poses:
    print("Testing pose", pos)
    qs_estimate = arm.inv_kin(pos[:3], pos[3])
    pos_estimate = arm.fkin(qs_estimate)
    arm.set_joint_targets(qs_estimate)
    #time.sleep(5)
    #pos_measured = arm.get_gripper_pos()

    print("Position estimate is", pos_estimate)
    print("Position measured is", pos_measured)
    print("qs estimate is:", qs_estimate)
    for a, b in zip(pos[0:3], pos_estimate):
        if math.fabs(a - b) > epsilon:
            raise Exception("ikin and fkin exceed error threshold!")
    print("--- NEXT ---")

print("All ok :)")


# for pos in [
#     [0,0,0,0],
#     [45,0,0,0],
#     [0,45,0,0],
#     [0,0,45,0],
#     [0,0,0,45],
#     [0,45,0,45],
#     [45,0,45,0],

#     ]:
#     print(arm.fkin(pos))
#     arm.set_joint_targets(pos)
#     time.sleep(5)
#     print(arm.get_joint_angles())
#     print(arm.get_gripper_pos())
#     print("-----")


print("fkin", arm.fkin([0,0,0,0]))
arm.set_joint_targets([0,0,0,0])
time.sleep(5)
print(arm.get_gripper_pos())

#j2 range -60...60
