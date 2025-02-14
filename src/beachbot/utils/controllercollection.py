
import numbers
from typing import Tuple, Union

class PIDController:
    def __init__(self, setpoint_x: float, setpoint_y: float, kp: Union[float, Tuple[float]], ki: Union[float, Tuple[float]]=0, kd: Union[float, Tuple[float]]=0):
        """Initialize the PID controller for 2d coordinates (x,y).

        Args:
            setpoint_x (float): Initial x target, in relative position (0..1)
            setpoint_y (float): Initial y target, in relative position (0..1)
            kp (Union[float, Tuple[float]]): Proportial gain, one value for doth dimensions or tuple of values (kp_x, kp_y)
            ki (Union[float, Tuple[float]], optional): Integral gain, one value for doth dimensions or tuple of values (ki_x, ki_y). Defaults to 0. TODO NOT IMPLMENTED.
            kd (Union[float, Tuple[float]], optional): Deriviative gain, one value for doth dimensions or tuple of values (kd_x, kd_y). Defaults to 0. TODO NOT IMPLMENTED.
        """
        
        # target position
        self.setpoint_x = setpoint_x
        self.setpoint_y = setpoint_y
        
        # Proportional factor for error correction can be one value or tuple (x,y)
        self.setPropotionalGain(kp)

        ## optional, default: set to 0 
        ## ki -> integration of error
        ## kd -> deriviative of error
        self.setIntegralGain(ki)
        self.setDeriviativeGain(kd)



        ## variables to calculate integral (sum of previous errors) and deriviative (difference to previous error)
        # self.integral_x = 0
        # self.integral_y = 0
        self.prev_error_x = 0
        self.prev_error_y = 0

    def setPropotionalGain(self, kp: Union[float, Tuple[float]]):
        if isinstance(kp, numbers.Number): kp = (kp,kp)
        self.kp : Tuple[float] = kp

    def setIntegralGain(self, ki: Union[float, Tuple[float]]):
        if isinstance(ki, numbers.Number): ki = (ki,ki)
        self.ki : Tuple[float] = ki

    def setDeriviativeGain(self, kd: Union[float, Tuple[float]]):
        if isinstance(kd, numbers.Number): kd = (kd,kd)
        self.kd : Tuple[float] = kd

    def get_output(self, x: float, y: float, debug:bool = False) -> Tuple[int, int]:
        """Update controller state given new measurement and return control output.
        Values are bounded to +/-100 (percent), and rounded to integer values.


        Args:
            x (float): Current x position (measurement), in relative position (0..1)
            y (float): Current y position (measurement), in relative position (0..1)
            debug (bool, optional): Print verbose debug information to stdout. Defaults to False.

        Returns:
            Tuple[int, int]: Estimated control value (x_out, y_out) in range -100..100 (percent)
        """
        error_x = self.setpoint_x - x
        error_y = self.setpoint_y - y

        if debug:
            print("PID error:", error_x, error_y)

        # Calculate proportional error signal:
        output_x = self.kp[0] * error_x
        output_y = self.kp[1] * error_y

        if debug:
            print("PID output:", output_x, output_y)


        ## Optional: Calculate integration error
        # self.integral_x += error_x
        # self.integral_y += error_y
        # output_x += self.ki * self.integral_x
        # output_y += self.ki * self.integral_y

        ## Optional calculate deriviative error
        # derivative_x = error_x - self.prev_error_x
        # derivative_y = error_y - self.prev_error_y
        # output_x += self.kd * derivative_x
        # output_y += self.kd * derivative_y


        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Truncate output to -100 to 100
        output_x = max(min(output_x, 100), -100)
        output_y = max(min(output_y, 100), -100)

        return round(output_x), round(output_y)
