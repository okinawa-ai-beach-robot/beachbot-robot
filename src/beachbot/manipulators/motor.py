

class Motor():
    def __init__(self, name: str,):
        self.name: str = name
        self.speed: float = 0

    
    def change_speed(self, speed: int) -> None:
        if not -100 <= speed <= 100:
            raise ValueError(
                f"Speed must be between -100 and 100 inclusive. Received: {speed}"
            )
        if speed<1 and speed>-1:
            speed=0
        self.speed = speed

    def get_speed(self):
        return self.speed

    def turn_off(self) -> None:
        self.change_speed(0)

    def cleanup(self):
        self.turn_off()