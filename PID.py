class PIDController:
    def __init__(
        self, kp: float, ki: float, kd: float, setpoint_x: float, setpoint_y: float
    ):
        self.kp = kp
        # self.ki = ki
        # self.kd = kd
        self.setpoint_x = setpoint_x
        self.setpoint_y = setpoint_y
        # self.prev_error_x = 0
        # self.prev_error_y = 0
        # self.integral_x = 0
        # self.integral_y = 0

    def get_output(self, x: float, y: float) -> Tuple[int, int]:
        error_x = self.setpoint_x - x
        error_y = self.setpoint_y - y

        # self.integral_x += error_x
        # self.integral_y += error_y

        # derivative_x = error_x - self.prev_error_x
        # derivative_y = error_y - self.prev_error_y

        output_x = (
            self.kp * error_x
        )  # + self.ki * self.integral_x + self.kd * derivative_x
        output_y = (
            self.kp * error_y
        )  # + self.ki * self.integral_y + self.kd * derivative_y

        # self.prev_error_x = error_x
        # self.prev_error_y = error_y

        # Truncate output to -100 to 100
        output_x = max(min(output_x, 100), -100)
        output_y = max(min(output_y, 100), -100)

        return int(output_x), int(output_y)

