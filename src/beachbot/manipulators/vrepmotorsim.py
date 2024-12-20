from .motor import Motor
from ..utils.vrepsimulation import vrep

class VrepMotorSim(Motor):
    def __init__(self, name:str, vrep_sim, max_velocity=5):
        super().__init__(name)
        self.vrep_sim = vrep_sim
        self._max_velocity=max_velocity
        self._init_vrep()

    @vrep
    def _init_vrep(self):
        self._motor_id = self.vrep_sim.getObject("/"+self.name)

    @vrep    
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
        # start moving at 15% power (as real platform has friction), if lower than 15% -> do not move
        if speed<15 and speed>-15:
            speed=0
        elif speed<=-15:
                speed += 15
        elif speed>=15:
             speed -= 15

        self.vrep_sim.setJointTargetVelocity(self._motor_id, self._max_velocity * speed / 100)
        