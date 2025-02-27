from beachbot.robot import RobotInterface, VrepRobotSimV1
from beachbot.ai import BlobDetectorOpenCV
import time

robot = VrepRobotSimV1()
image = robot.get_camera_image()
time.sleep(2.0)
image = robot.get_camera_image()
detector = BlobDetectorOpenCV()
boxes = detector.apply_model(image)
print(boxes)