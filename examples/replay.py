import argparse
from beachbot.robot.vreprobotsimv1 import VrepRobotSimV1
from beachbot.robot.jetsonrobotv1 import JetsonRobotV1


def main():
    parser = argparse.ArgumentParser(description="Run the robot or simulation.")
    parser.add_argument(
        "--mode",
        choices=["robot", "sim"],
        default="robot",
        help="Choose the mode: 'robot' (default) or 'sim'.",
    )
    args = parser.parse_args()

    if args.mode == "sim":
        print("Running in simulation mode.")
        robot = VrepRobotSimV1(scene="roarm_m1_recorder_3finger.ttt")
    else:
        print("Running in robot mode.")
        # Replace this with the actual robot initialization when available.
        robot = JetsonRobotV1()

    robot.arm.pickup()
    robot.arm.toss()


if __name__ == "__main__":
    main()
