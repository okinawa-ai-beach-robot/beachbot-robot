import time
from beachbot.config import logger
import threading


def sign(x):
    return (x > 0) - (x < 0)

def bounded(val, mi=0, ma=1):
    return min(ma, max(val, mi))


class DifferentialDrive(threading.Thread):
    def __init__(self, motor_left, motor_right, update_freq=100, command_timeout=1.0) -> None:
        # Init superclass thread
        super().__init__()
        # Do not block on exit (TODO)
        self.daemon = True

        self.motor_left = motor_left
        self.motor_right = motor_right
        self.update_freq = update_freq
        self._is_running = False
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.update_freq = update_freq
        self._is_running = False

        self._target_angular_vel = 0
        self._target_velocity = 0
        self._current_angular_vel = 0
        self._current_velocity = 0
        self._target_angular_vel = 0
        self._target_velocity = 0
        self._current_angular_vel = 0
        self._current_velocity = 0

        self._max_rate_of_change = 100

        self._motor_left_speed = 0
        self._motor_right_speed = 0
        self._motor_left_speed = 0
        self._motor_right_speed = 0

        self.motor_left.change_speed(self._motor_left_speed)
        self.motor_right.change_speed(self._motor_right_speed)

        self._last_command_update=time.time()
        self._last_command_overwrite=-1
        self._command_timeout=command_timeout


        super().start()

    def cleanup(self):
        self._is_running = False
        time.sleep(1.0 / self.update_freq)
        self.motor_left.change_speed(0)
        self.motor_right.change_speed(0)
        self.motor_left.cleanup()
        self.motor_right.cleanup()
        try:
            GPIO.cleanup()
        except Exception as ex:
            logger.error("GPIO cleanup failed (bug in GPIO?)")

    def run(self):
        self._is_running = True
        while self._is_running:
            t_start = time.time()
            # do work....

            # safety stop:
            if self._command_timeout is not None and self._command_timeout>0 and (self._target_angular_vel!=0 or self._target_velocity!=0):
                td_last_command = t_start-self._last_command_update
                if td_last_command>self._command_timeout:
                    # timeout, no command recieved for self._command_timeout seconds -> stop robot movement
                    self.set_target(0, 0)


            # Update current angular and linear velocity
            dir_delta = self._target_angular_vel - self._current_angular_vel
            vel_delta = self._target_velocity - self._current_velocity
            

            dir_dir = sign(dir_delta)
            vel_dir = sign(vel_delta)

            dir_dot = dir_dir * min(
                self._max_rate_of_change / self.update_freq, abs(dir_delta)
            )
            vel_dot = vel_dir * min(
                self._max_rate_of_change / self.update_freq, abs(vel_delta)
            )
            dir_dot = dir_dir * min(
                self._max_rate_of_change / self.update_freq, abs(dir_delta)
            )
            vel_dot = vel_dir * min(
                self._max_rate_of_change / self.update_freq, abs(vel_delta)
            )

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

            if self._motor_left_speed != int(__motor_left_speed):
                self._motor_left_speed = int(__motor_left_speed)
                self.motor_left.change_speed(self._motor_left_speed)

            if self._motor_right_speed != int(__motor_right_speed):
                self._motor_right_speed = int(__motor_right_speed)
                self.motor_right.change_speed(self._motor_right_speed)


            update_interval = (1.0 / self.update_freq)
            if self._last_command_overwrite>0: self._last_command_overwrite -= update_interval

            t_end = time.time()
            t_wait = update_interval - (t_end - t_start)
            if t_wait > 0:
                time.sleep(t_wait)
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


