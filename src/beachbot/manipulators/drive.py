from math import sqrt
import time
from beachbot.config import logger
from beachbot.manipulators.motor import Motor
from beachbot.utils.properties import HasProperties
import threading


def safe_division(numerator, denominator):
    """Return 0 if denominator is 0."""
    return denominator and numerator / denominator

def sign(x):
    """Calculate sign of number. returns 0 if input is zero.

    Args:
        x (number): input value

    Returns:
        number: -1  if x<0
                 1  if x>0
                 0  else
    """
    return (x > 0) - (x < 0)

def bounded(val, mi=0, ma=1):
    return min(ma, max(val, mi))

class DriveSystem(HasProperties):
    def __init__(self):
        super().__init__()

class DifferentialDrive(DriveSystem, threading.Thread):
    def __init__(self, motor_left:Motor, motor_right:Motor, update_freq=25, command_timeout=1.0) -> None:
        # Init superclass thread
        super().__init__()

        # Do not block on exit (TODO)
        self.daemon = True

        self.motor_left : Motor = motor_left
        self.motor_right : Motor = motor_right
        self.update_freq = update_freq
        self._is_running = False


        self._target_angular_vel = 0
        self._target_velocity = 0



        # The maximum value change per second of the contorl values (set_target)..
        # Values from set_target are gradually reached instaed of instantanously 
        # Meant to avoid quick back-forth movements or other unrealistic accelerations by the robot
        self.max_rate_of_change = 400

        # compensation of motor friction:
        # scale 0-100% requested motor speed to pwm duty cycle self.friction_compensation-100 percent, motor speed of 0 results in 0 percent pwm duty cycle
        # must be between 0 and <100
        self.friction_compensation=15

        self._motor_left_speed = 0
        self._motor_right_speed = 0
        self._target_motor_left_speed = 0
        self._target_motor_right_speed = 0



        self.motor_left.change_speed(self._motor_left_speed)
        self.motor_right.change_speed(self._motor_right_speed)

        self._last_command_update=time.time()
        self._last_command_overwrite=-1
        self._command_timeout=command_timeout


        self.register_property("max_rate_of_change", min_value=10, max_value=1000, descr="DifferentialDrive: The maximum rate of change of the control variables (rotation and velocity) per second. Acts as low pass filter for set_target(rot,vel) and avoids motor burn-out.")
        self.register_property("friction_compensation", min_value=0, max_value=50, descr="compensation of motor friction, scale 0-100% requested motor speed to pwm duty cycle self.friction_compensation-100 percent, motor speed of 0 results in 0 percent pwm duty cycle")

        super().start()

    def cleanup(self):
        self._is_running = False
        time.sleep(1.0 / self.update_freq)
        self.motor_left.change_speed(0)
        self.motor_right.change_speed(0)
        self.motor_left.cleanup()
        self.motor_right.cleanup()


    def run(self):
        self._is_running = True
        last_dimediff = 1/self.update_freq
        while self._is_running:
            t_start = time.time()
            # do work.... run the control loop

            # safety stop:
            if self._command_timeout is not None and self._command_timeout>0 and (self._target_angular_vel!=0 or self._target_velocity!=0):
                td_last_command = t_start-self._last_command_update
                if td_last_command>self._command_timeout:
                    # timeout, no command recieved for self._command_timeout seconds -> stop robot movement
                    self.set_target(0, 0)



            # Estimate motor velocity based on angluar and linear velocity (unbounded):
            _target_motor_left_speed = self._target_velocity + 0.5 * self._target_angular_vel
            _target_motor_right_speed = self._target_velocity - 0.5 * self._target_angular_vel
            

            # def f1(l,a):
            #     m1 = l + 0.5*a
            #     m2 = l - 0.5*a
            #     desired_power = min(sqrt(a**2 + l**2),100)
            #     unscaled_max_power = max(abs(m1), abs(m2))
            #     scale_fac = safe_division(desired_power, unscaled_max_power)
            #     m1 *= scale_fac
            #     m2 *= scale_fac





            # As we do not have velocity control (we control power, pwm duty cycle for now)
            # we scale desired output such that total motor power is scaled in relation
            # to euclidean distance of 2d control input (velocity and angular velocity) to its neutral point (0,0) 
            # So distance of the joystick form center controls max motor power
            # Range can be 0-100 percent max motor power:
            #
            # 1. desired_power based on, e.g. distance of joystick form center:
            desired_power = min(sqrt(self._target_angular_vel**2 + self._target_velocity**2),100)
            # 2. scaling to bound motor power to 100, but also to keep relative mottor values through scaling:
            unscaled_max_power = max(abs(_target_motor_right_speed), abs(_target_motor_left_speed))
            # 3. apply both scaling factors, also consider case unscaled_max_power=0 (velocity=0 and anglular_velocity=0) by 'save_division'
            scale_fac = safe_division(desired_power, unscaled_max_power)
            _target_motor_left_speed *= scale_fac
            _target_motor_right_speed *= scale_fac


            


            # Rescale requested _target_motor_right_speed and _target_motor_left_speed to be in range self.friction_compensation..100 (positive or negative)
            # if speed if zero, no change occurs (due to sign function)
            _target_motor_left_speed = sign(_target_motor_left_speed) * self.friction_compensation + _target_motor_left_speed * (100-self.friction_compensation)/100
            _target_motor_right_speed = sign(_target_motor_right_speed) * self.friction_compensation + _target_motor_right_speed * (100-self.friction_compensation)/100


            # store in class, e.g. for plotting:
            self._target_motor_left_speed = _target_motor_left_speed
            self._target_motor_right_speed = _target_motor_right_speed



            # change requested motor speed, limited through max rate of change:
            # 1. estimate difference between current (last) motor command and new targets
            motor_left_delta = _target_motor_left_speed - self._motor_left_speed
            motor_right_delta = _target_motor_right_speed - self._motor_right_speed
            # 2. estimate sign/direction of requested change in motor speed
            motor_left_dir = sign(motor_left_delta)
            motor_right_dir = sign(motor_right_delta)
            # 3. estimate necessary change of motor values
            motor_left_dot = motor_left_dir * min(last_dimediff*self.max_rate_of_change, abs(motor_left_delta))
            motor_right_dot = motor_right_dir * min(last_dimediff*self.max_rate_of_change, abs(motor_right_delta))
            # 4. update motor speed and bound to valid value ranges
            __motor_left_speed = bounded(self._motor_left_speed + motor_left_dot, -100, 100)
            __motor_right_speed = bounded(self._motor_right_speed + motor_right_dot, -100, 100)



            # Update motors if target values changed
            if self._motor_left_speed != int(__motor_left_speed):
                self._motor_left_speed = int(__motor_left_speed)
                self.motor_left.change_speed(self._motor_left_speed)

            if self._motor_right_speed != int(__motor_right_speed):
                self._motor_right_speed = int(__motor_right_speed)
                self.motor_right.change_speed(self._motor_right_speed)



            # The following code causes the control loop to keep the desired update rate "self.update_freq"
            update_interval = (1.0 / self.update_freq)
            if self._last_command_overwrite>0: self._last_command_overwrite -= update_interval

            t_end = time.time()
            t_wait = update_interval - (t_end - t_start)
            if t_wait > 0:
                time.sleep(t_wait)
            # Simulation may slows down update loop, but does not matter, update loop uses real measured time last_dimediff for update:
            # elif t_wait < 0:
            #    logger.warning(f"Drive System control loop out of time, took {(t_end - t_start)}sec, target loop is {update_interval}sec")
            last_dimediff = (time.time() - t_start)
                
        # Cleanup, end control loop, stop motors :)
        self.motor_left.change_speed(0)
        self.motor_right.change_speed(0)

    def set_target(self, angular_vel=0, velocity=0, force_overwrite:float = 0):
        if self._last_command_overwrite>0 and not force_overwrite>0:
            # ignore input commands if a previous target command forced an overwrite for a certain amount
            # self._last_command_overwrite seconds left until we are allowed to send a command again....
            return
        
        self._last_command_update = time.time()
        self._target_angular_vel = angular_vel
        self._target_velocity = velocity

        if force_overwrite>0:
            # request blocking if input commands for force_overwrite seconds
            # Use case: e.g. manual command inputs block all other motor commands for a certain amount of time
            self._last_command_overwrite=force_overwrite


