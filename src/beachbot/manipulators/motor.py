
class Motor:
    def __init__(
        self,
        name: str
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


    def turn_off(self) -> None:
        self.change_speed(0)

    def cleanup(self):
        self.turn_off()
