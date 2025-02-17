import time
from beachbot.config import logger
from beachbot.manipulators.motor import Motor
from beachbot.utils.properties import HasProperties
import threading


def sign(x):
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
        self._current_angular_vel = 0
        self._current_velocity = 0


        # The maximum value change per second of the contorl values (set_target)..
        # Values from set_target are gradually reached instaed of instantanously 
        # Meant to avoid quick back-forth movements or other unrealistic accelerations by the robot
        self.max_rate_of_change = 100

        self._motor_left_speed = 0
        self._motor_right_speed = 0


        self.motor_left.change_speed(self._motor_left_speed)
        self.motor_right.change_speed(self._motor_right_speed)

        self._last_command_update=time.time()
        self._last_command_overwrite=-1
        self._command_timeout=command_timeout


        self.register_property("max_rate_of_change", min_value=10, max_value=1000, descr="DifferentialDrive: The maximum rate of change of the control variables (rotation and velocity) per second. Acts as low pass filter for set_target(rot,vel) and avoids motor burn-out.")


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


            # estimate difference between current (last) motor command and targets (requested via set_target)
            # dir menas how fast robot should rotate around own axis
            # vel means how fast the robot should move forward
            # See "set_targets" for more info
            dir_delta = self._target_angular_vel - self._current_angular_vel
            vel_delta = self._target_velocity - self._current_velocity
            
            # Estimate the sign, i.e. is the new targets are smaller or lager as the current ones (-1 or 1)
            dir_dir = sign(dir_delta)
            vel_dir = sign(vel_delta)

            # Move current motor signal toward the target, but limit the amount of value change to "self.max_rate_of_change" 
            dir_dot = dir_dir * min(
                last_dimediff*self.max_rate_of_change, abs(dir_delta)
            )

            vel_dot = vel_dir * min(
                last_dimediff*self.max_rate_of_change, abs(vel_delta)
            )

            # Limit calue rage to -100 to 100 percent
            self._current_angular_vel = bounded(
                self._current_angular_vel + dir_dot, -100, 100
            )

            self._current_velocity = bounded(
                self._current_velocity + vel_dot, -100, 100
            )

            # Estimate motor velocity based on angluar and linear velocity and update motors if changed:
            __motor_left_speed = bounded(
                self._current_velocity + 0.5 * self._current_angular_vel, -100, 100
            )

            __motor_right_speed = bounded(
                self._current_velocity - 0.5 * self._current_angular_vel, -100, 100
            )

            # +/-15 percent no motion ...
            __motor_left_speed = bounded(
                sign(__motor_left_speed) * 15 + __motor_left_speed, -100, 100
            )
            __motor_right_speed = bounded(
                sign(__motor_right_speed) * 15 + __motor_right_speed, -100, 100
            )

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


