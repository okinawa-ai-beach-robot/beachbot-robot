import time
import numpy as np
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm
timestep = 0.2
increment = 0.025


def record_to_npz(filename, timestamps, qs, taus):
    np.savez(
        filename,
        timestamps=np.array(timestamps),
        qs=np.array(qs),
        taus=np.array(taus),
    )


def pickup(log_file):
    pathpospercent = 0.25
    i = -increment
    start_time = time.time()
    timestamps = []
    qs = []
    taus = []

    while pathpospercent > 0.0:
        current_time = time.time()
        q, tau = simarm.get_joint_state()
        timestamps.append(current_time - start_time)  # Relative timestamp
        qs.append(q)
        taus.append(tau)
        time.sleep(timestep)
        pathpospercent += i
        simarm.set_target_path_pos(percent=pathpospercent, offset=[0, 0, 0])

    # Ensure the gripper matches the target state
    q[4] = 1
    simarm.set_joint_targets(q)
    time.sleep(2)

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)

    record_to_npz(log_file, timestamps, qs, taus)


def toss(log_file):
    pathpospercent = 0.0
    i = increment
    start_time = time.time()
    timestamps = []
    qs = []
    taus = []

    while pathpospercent < 1.0:
        current_time = time.time()
        q, tau = simarm.get_joint_state()
        timestamps.append(current_time - start_time)  # Relative timestamp
        qs.append(q)
        taus.append(tau)
        time.sleep(timestep)
        pathpospercent += i
        simarm.set_target_path_pos(percent=pathpospercent, offset=[0, 0, 0])

    # Ensure the gripper matches the target state
    q[4] = 0
    simarm.set_joint_targets(q)
    time.sleep(3)

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)

    record_to_npz(log_file, timestamps, qs, taus)


simarm.go_home()
pickup("pickup_log.npz")
toss("toss_log.npz")
