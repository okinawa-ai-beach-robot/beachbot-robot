from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")

robot.arm.pickup()
robot.arm.toss()
