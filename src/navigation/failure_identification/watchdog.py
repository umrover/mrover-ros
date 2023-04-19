import numpy as np
import rospy
from pandas import DataFrame


class WatchDog:
    def __init__(self):
        pass

    def is_stuck(self, dataframe: DataFrame):
        return True
