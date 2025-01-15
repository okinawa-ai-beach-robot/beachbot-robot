from time import sleep, time
import numpy as np
try:
    from importlib.resources import files  # Python 3.9+
except ImportError:
    from importlib_resources import files  # Backport for Python 3.8


class Trajectory:
    def __init__(self, ts=None, qs=None, taus=None):
        self.ts = ts
        self.qs = qs
        self.taus = taus

    @classmethod
    def from_file(cls, trajectory_path: str):
        """Class method to create a Trajectory instance from a file."""
        data = np.load(trajectory_path)
        return cls(ts=data['ts'], qs=data['qs'], taus=data['taus'])


def load_trajectory(trajectory: str):
    trajectory_path = files('beachbot.assets').joinpath(trajectory)
    data = np.load(trajectory_path)
    ts = data["ts"]
    qs = data["qs"]
    taus = data["taus"]
    return ts, qs, taus


def load_default_trajectories():
    global pickup_trajectory, toss_trajectory
    pickup_trajectory = Trajectory.from_file('beachbot.assets/pickup.npz')
    toss_trajectory = Trajectory.from_file('beachbot.assets/toss.npz')


def replay_trajectory(self,
                      qs: np.ndarray,
                      ts: np.ndarray = None,
                      timestep_length: float = 0.01):
    """
    Replay a trajectory.

    Parameters:
    - qs: numpy.ndarray, shape (n, m), where n is the number of time steps and m is the number of joints.
    - ts: numpy.ndarray, shape (n,), optional, timestamps for each time step.
    - timestep_length: float, optional, length of each time step in seconds. Will default to 0.01 if ts is not provided.
    """

    # Set initial position as initial timestamp should be around 0
    self.set_joint_targets(qs[0])
    self.wait_target_arrival()
    sleep_time = 0

    if ts is None:
        for t in range(1, qs.shape[0]):
            self.set_joint_targets(qs[t])
            sleep_time = timestep_length
            sleep(sleep_time)
    else:
        for t in range(1, qs.shape[0]):
            self.set_joint_targets(qs[t])
            sleep_time = ts[t] - ts[t-1]
            sleep(sleep_time)


def wait_target_arrival(self, max_distance=0.02, timeout=10, polling_interval=0.1):
    """
    Wait until the gripper position is within the target range or timeout.

    Parameters:
    - max_distance: float, maximum allowable distance to the target.
    - timeout: float, maximum time to wait in seconds.
    - polling_interval: float, time between position checks.

    Returns:
    - bool: True if the target was reached, False if timed out.
    """

    t_start = time()
    while True:
        pos = self.get_gripper_pos()
        target = self.get_gripper_target()
        if np.linalg.norm(pos - target) <= max_distance:
            return True
        if time() - t_start > timeout:
            return False
        sleep(polling_interval)


def pickup(self):
    self.replay_trajectory(pickup_trajectory)


def toss(self):
    self.replay_trajectory(toss_trajectory)


pickup_trajectory: Trajectory = None
toss_trajectory: Trajectory = None
load_default_trajectories()
