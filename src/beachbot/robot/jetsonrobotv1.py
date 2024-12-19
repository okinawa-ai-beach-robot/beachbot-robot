from ..sensors.jetsoncsicameraopencv import JetsonCsiCameraOpenCV
from ..sensors.usbcameraopencv import UsbCameraOpenCV
from ..manipulators.drive import DifferentialDrive
from ..manipulators.jetsonmotor import JetsonMotor
from ..manipulators.roarmm1 import RoArmM1
from .robotinterface import RobotInterface

try:
    import Jetson.GPIO as GPIO
except (RuntimeError) as ex:
    raise RuntimeError("You are not on a Jetson and are attempting to access Jetson GPIO. If you intended to use simulator use --sim flag.")

class JetsonRobotV1(RobotInterface):
    def __init__(self):
        super().__init__()
        # Camera Setup:
        self.cameradevices[RobotInterface.CAMERATYPE.FRONT] = JetsonCsiCameraOpenCV()
        try:
            usbcam = UsbCameraOpenCV()
            self.cameradevices[RobotInterface.CAMERATYPE.GRIPPER] = usbcam
        except Exception:
            pass

        # Motor Hardware Config:
        pwm_pins = [32, 33]
        gpio_pins = [15, 7, 29, 31]
        _frequency_hz = 5000
        GPIO.setmode(GPIO.BOARD)

        # Motor Controller Setup:
        motor_left = JetsonMotor("motor_left", pwm_pins[0], gpio_pins[0], gpio_pins[1], _frequency_hz)
        motor_right = JetsonMotor("motor_right", pwm_pins[1], gpio_pins[2], gpio_pins[3], _frequency_hz)
        self.platform = DifferentialDrive(motor_left, motor_right)

        # Init Robot arm, gripper limits [q_open, q_close] must be adjusted to gripper hardware!
        self.arm = RoArmM1(gripper_limits=[50,60])

    def stop(self):
        print("TODO: Stop robot")
    
    
    
