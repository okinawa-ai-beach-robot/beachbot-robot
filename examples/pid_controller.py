from beachbot import manipulators
from beachbot import utils
import time
from yolov5.models.common import Detections
import pandas as pd


def get_highest_confidence_trash_easy_center(detections: Detections) -> dict:
    """
    Get the xcenter and ycenter of the bounding box for the highest confidence "trash_easy" prediction.

    Args:
    - detections ('Detections'): An instance of the Detections class containing the object detection results.

    Returns:
    - coords (dict): A dictionary containing the xcenter and ycenter coordinates, or None if no detections are found.
    """
    if len(detections.pred) == 0:
        print("No detections found.")
        return None

    # Get pandas dataframe for xyxy predictions
    df: pd.DataFrame = detections.pandas().xywhn[0]

    # Filter predictions by class name "trash_easy"
    trash_easy_df: pd.DataFrame = df[df["name"] == "trash_easy"]

    if len(trash_easy_df) == 0:
        print("No 'trash_easy' detections found.")
        return None

    # Get the row with the highest confidence
    highest_confidence_row: pd.Series = trash_easy_df.loc[
        trash_easy_df["confidence"].idxmax()
    ]

    coords = dict(
        x=highest_confidence_row["xcenter"],
        y=highest_confidence_row["ycenter"],
    )

    return coords


def main():
    kp = 1.0
    ki = 0.0
    kd = 0.0
    setpoint_x = 0.0
    setpoint_y = 0.0
    pid_controller = utils.controllercollection.PIDController(setpoint_x, setpoint_y, kp, ki, kd)

    # Example Motor setup and initialization
    # TODO get default pin values from some beachbot.util definition
    motor_left = manipulators.JetsonMotor(pwm_pin=18, in1=23, in2=24, frequency_hz=100)
    motor_right = manipulators.JetsonMotor(pwm_pin=25, in1=8, in2=7, frequency_hz=100)

    # Load model
    # TODO
    model = None
    # Load image or pull from camera
    # TODO
    img = None

    # Main loop
    while True:
        # Get the center of the 'trash_easy' object
        detections = model(img)
        center: dict = utils.get_highest_confidence_trash_easy_center(detections)

        if not center:
            continue

        # Get PID output
        pid_output_x, pid_output_y = pid_controller.get_output(center["x"], center["y"])
        print("PID output:", pid_output_x, pid_output_y)

        # Map PID output to motor speeds (-100 to 100)
        speed_left = pid_output_x
        speed_right = pid_output_x

        # Drive motors
        motor_left.change_speed(speed_left)
        motor_right.change_speed(speed_right)

        time.sleep(0.1)  # Adjust as needed for your application
