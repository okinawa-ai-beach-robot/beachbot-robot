import argparse




def main():
    parser = argparse.ArgumentParser(description="Run the robot or simulation.")
    parser.add_argument(
        "--mode",
        choices=["robot", "sim", "usb"],
        default="robot",
        help="Choose the mode: 'robot' (default) or 'sim' or usb (only arm availabe/connected via usb).",
    )
    args = parser.parse_args()

    if args.mode == "sim":
        print("Running in simulation mode.")
        from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
        robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
    elif args.mode == "usb":
        print("Running in usb mode.")
        from beachbot.manipulators.roarmm1 import RoArmM1_Custom3FingerGripper
        # Replace this with the actual robot initialization when available.
        class robotmp:
            arm = RoArmM1_Custom3FingerGripper()
        robot = robotmp()

    else:
        print("Running in robot mode.")
        from beachbot.robot.jetsonrobotv1 import JetsonRobotV1
        # Replace this with the actual robot initialization when available.
        robot = JetsonRobotV1()

    robot.arm.pickup()
    robot.arm.toss()


if __name__ == "__main__":
    main()
