from drive import collector
import pandas as pd
from util.np_utils import euclidean_distance

# 10 or more data objects flagged as stuck, then you are actually stuck
STUCK_THRESHOLD = 10
ANGULAR_THRESHOLD = 0.001
LINEAR_THRESHOLD = 0.001

#Have a variable to keep track of the last row
class WatchdogBase:
    def __init__(self):
        pass

    # check if 10 or more consecutive objects in the self.stuck list get flagged
    # as stuck and if they do enter the recovery sequence
    # Returns bool
    def is_stuck(self) -> bool:
        return True

    """
    def is_stuck(self, data_obj: Data) -> bool:
        self.history.append(data_obj)
        if len(self.history) < STUCK_THRESHOLD:
            return False

        if len(self.stuck_list) >= STUCK_THRESHOLD:
            self.clear_history()
            self.stuck_list.clear()
            return True
        decision = self.evaluate_stuck()
        if decision:
            self.stuck_list.append(decision)
        return False
    """

    def evaluate_stuck(self):
        pass

    def clear_history(self):
        self.history.clear()

    def recover(self):
        self.stuck_list.clear()


    

# Each child class will have a different is_stuck function which evaluate whether the rover is stuck differently
# Each child class will have its own history of data objects
class WatchdogSimpleLinear(WatchdogBase):
    def __init__(self):
        super().__init__()

    # compare actual and commanded velocities
    # for each element in the history, if the commanded and angular velocities are different
    # append true to the stuck_list
    def evaluate_stuck(self):
        for data in super().history:
            if euclidean_distance(data.commanded_linear_vel, data.actual_linear_vel) < LINEAR_THRESHOLD:
                super().stuck_list.append(True)


class WatchdogSimpleAngular(WatchdogBase):
    def __init__(self):
        super().__init__()

    def evaluate_stuck(self):
        for data in super().history:
            if abs(data.commanded_angular_vel[3] - data.actual_angular_vel) < ANGULAR_THRESHOLD:
                super().stuck_list.append(True)


class WatchdogWheel(WatchdogBase):
    def __init__(self):
        super().__init__()

    # compare actual velocities with effort and wheel velocities
    def evaluate_stuck(self):
        pass


class WatchdogOff(WatchdogBase):
    def __init__(self):
        super().__init__()

    # def is_stuck(self, data_obj: Data):
    #    return False
