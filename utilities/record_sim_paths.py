import time
import numpy as np
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from beachbot.manipulators.arm import normalize_gripper_angle
from pathlib import Path

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm


def record_to_npz(filename, timestamps, qs, taus):
    # create parent directories if they don't exist
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        filename,
        ts=np.array(timestamps),
        qs=np.array(qs),
        taus=np.array(taus),
    )


def pickup(trajectory_file, pathpospercent):
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
        simarm.wait_target_pos_arrival()

    # Ensure the gripper matches the target state
    simarm.close_gripper()

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)

    record_to_npz(trajectory_file, timestamps, qs, taus)


def toss(trajectory_file, pathpospercent):
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
        simarm.wait_target_pos_arrival()

    # Ensure the gripper matches the target state
    simarm.open_gripper()

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)
    record_to_npz(trajectory_file, timestamps, qs, taus)


increment = 0.01
pathpospercent = 0.25
simarm.open_gripper()
simarm.set_target_path_pos(pathpospercent)
simarm.wait_target_pos_arrival()
assets_path = "src/beachbot/assets/"
pickup_path = assets_path + "pickup.npz"
toss_path = assets_path + "toss.npz"
pickup(pickup_path, pathpospercent)
toss(toss_path, pathpospercent)
