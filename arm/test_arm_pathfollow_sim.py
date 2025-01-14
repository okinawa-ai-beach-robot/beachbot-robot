import time
import beachbot
from beachbot.robot.robotinterface import RobotInterface
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
import math

epsilon = 1e-5

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")

arm = robot.arm


arm.go_home()
arm.set_gripper(0)
time.sleep(5.0)
print(arm.get_gripper_pos())
time.sleep(2.0)

print("Activate inverse mode")
arm.set_target_path_pos(0.5)
for i in range(30):
    time.sleep(0.1)
    gripper_pos = arm.get_gripper_pos()
    print(gripper_pos)
    print("dist is:", math.sqrt((gripper_pos[0]-gripper_pos[3])**2 + (gripper_pos[1]-gripper_pos[4])**2+ (gripper_pos[2]-gripper_pos[5])**2))

arm.set_gripper(1.0)

