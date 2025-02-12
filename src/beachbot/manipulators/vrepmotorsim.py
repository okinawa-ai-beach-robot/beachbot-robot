from ..utils.vrepsimulation import vrep
from beachbot.manipulators.motor import Motor

class VrepMotorSim(Motor):
    def __init__(self, vrep_sim, motor_name, max_velocity=5):
        super().__init__(motor_name)
        self.vrep_sim = vrep_sim
        self._max_velocity=max_velocity
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
        super().change_speed(speed)

        # start moving at 15% power (as real platform has friction), if lower than 15% -> do not move
        if speed<15 and speed>-15:
            speed=0
        elif speed<=-15:
                speed += 15
        elif speed>=15:
             speed -= 15

        self.vrep_sim.setJointTargetVelocity(self._motor_id, self._max_velocity * speed / 100)

        