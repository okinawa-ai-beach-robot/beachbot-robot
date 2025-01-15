import time
import numpy as np
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm


def record_to_npz(filename, timestamps, qs, taus):
    np.savez(
        filename,
        timestamps=np.array(timestamps),
        qs=np.array(qs),
        taus=np.array(taus),
    )


def pickup(log_file, pathpospercent):
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
        pathpospercent += i
        simarm.set_target_path_pos(percent=pathpospercent)
        wait_target_arrival()

    # Ensure the gripper matches the target state
    q[4] = 1
    simarm.set_joint_targets(q)

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)

    record_to_npz(log_file, timestamps, qs, taus)


def toss(log_file, pathpospercent):
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
        pathpospercent += i
        simarm.set_target_path_pos(percent=pathpospercent, offset=[0, 0, 0])
        wait_target_arrival()

    # Ensure the gripper matches the target state
    q[4] = 0
    simarm.set_joint_targets(q)

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)

    record_to_npz(log_file, timestamps, qs, taus)


def wait_target_arrival():
    pos = simarm.get_gripper_pos()
    target = simarm.get_gripper_target()
    while np.linalg.norm(pos - target) > 0.02:
        time.sleep(0.1)
        pos = simarm.get_gripper_pos()
        target = simarm.get_gripper_target()


increment = 0.01
pathpospercent = 0.25
simarm.set_target_path_pos(pathpospercent)
wait_target_arrival()
pickup("pickup_log.npz", pathpospercent)
toss("toss_log.npz", pathpospercent)
