import numpy as np
import rospy
from pandas import DataFrame


class WatchDog:
    def __init__(self, collector_in):
        self.collector = collector_in

    def is_stuck(self, dataframe: DataFrame):
        return True
