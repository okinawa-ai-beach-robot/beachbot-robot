import time
import beachbot
import math
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1

robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
simarm = robot.arm


pathpos=0
try:
    while True:
        time.sleep(0.1)
        q,tau = simarm.get_joint_state()
        print("Q:", q)
        pathpos +=0.02
        if pathpos>2*math.pi:
            pathpos -= 2*math.pi
        pathpospercent = (math.sin(pathpos) + 1.0) / 2
        #simarm.set_target_path_pos(percent=pathpospercent, offset=[0,0,0.1]) 
        # offset=[0,0,0.1] -> we move the target 0.1 meter up, as the finger collide with ground during pickup -> nont reachable all the time

        ## Example: to perform an offset (only path pos=0 -> grasping) and keep same drop off position:
        x_offset = 0.01 # 0.01 meter sideways
        z_offset = 0.05 #lift it up close tot he pickup position, otherwise we collide gripper vs ground!
        offset_fac = 1-pathpospercent # 0percent -> full offset
        simarm.set_target_path_pos(percent=pathpospercent, offset=[offset_fac*x_offset, 0, offset_fac*z_offset])


except KeyboardInterrupt:
    print('Bye Bye!')

