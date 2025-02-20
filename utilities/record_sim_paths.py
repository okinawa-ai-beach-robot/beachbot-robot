import time
import numpy as np
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from pathlib import Path

timeout_sec=2
assets_path = "../src/beachbot/assets/"


def record_to_npz(filename, timestamps, qs, taus):
    # create parent directories if they don't exist
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        filename,
        ts=np.array(timestamps),
        qs=np.array(qs),
        taus=np.array(taus),
    )


def pickup(trajectory_file, pathpospercent, offset=[0,0,0], increment=0.01):
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
        pathpospercent = min(max(0,pathpospercent),1)
        #pathpospercent, zero means pickup position, 1 means toss poisition
        interpol_offset = [val*(1-pathpospercent) for val in offset]
        simarm.set_target_path_pos(percent=pathpospercent, offset=interpol_offset)
        simarm.wait_target_pos_arrival(timeout=timeout_sec)

    # Ensure the gripper matches the target state
    simarm.close_gripper()

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)

    record_to_npz(trajectory_file, timestamps, qs, taus)


def toss(trajectory_file, pathpospercent, offset=[0,0,0], increment=0.01):
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
        pathpospercent = min(max(0,pathpospercent),1)
        #pathpospercent, zero means pickup position, 1 means toss poisition
        interpol_offset = [val*(1-pathpospercent) for val in offset]
        simarm.set_target_path_pos(percent=pathpospercent, offset=interpol_offset)
        simarm.wait_target_pos_arrival(timeout=timeout_sec)

    # Ensure the gripper matches the target state
    simarm.open_gripper()

    # Record final state
    current_time = time.time()
    q, tau = simarm.get_joint_state()
    timestamps.append(current_time - start_time)
    qs.append(q)
    taus.append(tau)
    record_to_npz(trajectory_file, timestamps, qs, taus)






robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm

print("start")
increment = 0.01
pathpospercent = 0.25
simarm.open_gripper()
simarm.set_target_path_pos(pathpospercent)
simarm.wait_target_pos_arrival()
pickup_path = assets_path + "pickup.npz"
toss_path = assets_path + "toss.npz"

print("pickup")
pickup(pickup_path, pathpospercent, increment=increment)
print("toss")
toss(toss_path, pathpospercent, increment=increment)

print("Done recording default path")
robot.cleanup()


# record offset conditions for interpolation:
print("Start recording offset paths")
condition_dist = 0.10 # +/- 5cm

for cond in [(-condition_dist,-condition_dist),(-condition_dist,condition_dist),(condition_dist,-condition_dist),(condition_dist,condition_dist)]:
    x_off = cond[0]
    y_off = cond[1]
    offset_pickup = [x_off, y_off, 0]
    print(f"Record for condition [{x_off},{y_off}] offset of target....")
    #
    robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
    simarm = robot.arm
    simarm.open_gripper()
    simarm.set_target_path_pos(pathpospercent)
    simarm.wait_target_pos_arrival()
    
    pickup_path = assets_path + f"pickup_{x_off*100:.0f}_{y_off*100:.0f}cm.npz"
    toss_path = assets_path + f"toss_{x_off*100:.0f}_{y_off*100:.0f}cm.npz"
    pickup(pickup_path, pathpospercent, increment=increment, offset=offset_pickup)
    toss(toss_path, pathpospercent, increment=increment, offset=offset_pickup)
    robot.cleanup()

print(f"Done wit hall the fancy recrdings, stores in folder {assets_path}... see ya!")

