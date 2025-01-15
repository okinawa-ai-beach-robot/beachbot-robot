import numpy as np
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from beachbot.manipulators.arm import pickup_trajectory, toss_trajectory

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm

data = np.load("pickup.npz")

qs = data['qs']
taus = data['taus']
ts = data['ts']
simarm.replay_trajectory(qs, ts)
