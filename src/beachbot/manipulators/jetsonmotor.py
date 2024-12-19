from beachbot.config import logger

try:
    import Jetson.GPIO as GPIO
except (RuntimeError) as ex:
    raise RuntimeError("You are not on a Jetson and are attempting to access Jetson GPIO. If you intended to use simulator use --sim flag.")

import threading

class JetsonMotor:
    def __init__(
        self,
        name: str,
        pwm_pin: int,
        in1: int,
        in2: int,
        frequency_hz: int,
        lo1: int = None,
        lo2: int = None,
    ) -> None:
        """
        Initializes the Motor class.

        :param name: Name of the motor. Numbering just as 1 or 2 is fine. Used for logs
        :param pwm_pin: GPIO pin connected to the PWM control.
        :param in1: GPIO pin connected to IN1 on the motor driver.
        :param in2: GPIO pin connected to IN2 on the motor driver.
        :param frequency_hz: PWM frequency in hertz.
        :param lo1: Optional GPIO pin for LO1 (error signal from motor driver), should be between 0 and 40.
        :param lo2: Optional GPIO pin for LO2 (error signal from motor driver), should be between 0 and 40.
        """
        self.name: str = name
        self.pwm_pin: int = pwm_pin
        self.in1: int = in1
        self.in2: int = in2
        self.frequency_hz: int = frequency_hz
        self.lo1: int = lo1
        self.lo2: int = lo2
        self.duty_cycle_percent: int = (
            0  # Initialized to 0, and int can hold -100 to 100 inclusively
        )
        GPIO.setup([in1, in2], GPIO.OUT, initial=GPIO.LOW)
        # set pin as an output pin with optional initial state of LOW
        GPIO.setup(self.pwm_pin, GPIO.OUT, initial=GPIO.LOW)

        if lo1 is not None:
            GPIO.setup(lo1, GPIO.IN)
        if lo2 is not None:
            GPIO.setup(lo2, GPIO.IN)

        # p1 is a PWM object.
        self.pwm = GPIO.PWM(self.pwm_pin, self.frequency_hz)
        self.pwm.start(self.duty_cycle_percent)

        # Start the thread to poll LO1 and LO2 if they are provided
        if lo1 is not None and lo2 is not None:
            self.polling_thread = threading.Thread(target=self.poll_lo_pins)
            self.polling_thread.daemon = True
            self.polling_thread.start()

    def poll_lo_pins(self) -> None:
        """
        Polls the LO1 and LO2 pins and prints their status based on the truth table.
        """
        # Thermal shutdown counter. If above 5, throw runtime error
        thermal_shutdown_counter = 0
        while True:
            lo1_value = GPIO.input(self.lo1)
            lo2_value = GPIO.input(self.lo2)

            if lo1_value == GPIO.LOW and lo2_value == GPIO.LOW:
                status = "Detected over thermal (TSD)"
                thermal_shutdown_counter += 1
                if thermal_shutdown_counter > 5:
                    # Stop motor and throw runtime error to stop execution
                    self.turn_off()
                    raise RuntimeError(f"[{self.name}] Detected over thermal (TSD)")
            else:
                thermal_shutdown_counter = 0
                if lo1_value == GPIO.HIGH and lo2_value == GPIO.HIGH:
                    status = "Motor{ Normal status (Normal operation)"
                elif lo1_value == GPIO.HIGH and lo2_value == GPIO.LOW:
                    status = "Detected motor load open (OPD)"
                elif lo1_value == GPIO.LOW and lo2_value == GPIO.HIGH:
                    status = "Detected over current (ISD)"
                else:
                    status = "Unknown status"

            print(f"[{self.name}] LO1: {lo1_value}, LO2: {lo2_value}, Status: {status}")
            time.sleep(0.1)

    def change_speed(self, speed: int) -> None:
        """
        Change the motor speed.

        Args:
            speed (int): Speed of the motor, ranging from -100 to 100.
                         Negative values represent reverse motion,
                         positive values represent forward motion,
                         and zero stops the motor.
        """
        if not -100 <= speed <= 100:
            raise ValueError(
                f"Speed must be between -100 and 100 inclusive. Received: {speed}"
            )
        if speed<1 and speed>-1:
            speed=0


        self.duty_cycle_percent = abs(speed)

        if speed > 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            # Brake: INx1 = LOW, INx2 = LOW
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

        self.pwm.ChangeDutyCycle(self.duty_cycle_percent)

    def turn_off(self) -> None:
        self.change_speed(0)
        self.pwm.stop()

    def cleanup(self):
        self.turn_off()