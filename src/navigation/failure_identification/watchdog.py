import numpy as np
import rospy
from pandas import DataFrame

# 10 or more data objects flagged as stuck, then you are actually stuck
DF_THRESHOLD = 10
STUCK_THRESHOLD = 10
ANGULAR_THRESHOLD = 0.001
LINEAR_THRESHOLD = 0.001


class WatchDog:
    def __init__(self, collector_in):
        self.collector = collector_in

    def is_stuck(self, dataframe: DataFrame):
        return True

    # def check_angular_stuck(self):
    #     counter = 0
    #     while (self.collector.left_pointer < self.collector.right_pointer):
    #         actual = (self.collector._df.loc[self.collector._df['row'] == self.collector.left_pointer]['actual_angular_vel']).iloc[0]
    #         commanded = (self.collector._df.loc[self.collector._df['row'] == self.collector.left_pointer]['commanded_angular']).iloc[0]
    #         if(actual < ANGULAR_THRESHOLD and commanded[2] > 0):
    #             counter += 0
    #         self.collector.left_pointer += 1
    #     rospy.logerr(f"ANGULAR STUCK COUNTER: {counter}")
    #     return counter >= STUCK_THRESHOLD

    # def check_linear_stuck(self):
    #     counter = 0
    #     while (self.collector.left_pointer < self.collector.right_pointer):
    #         actual = (self.collector._df.loc[self.collector._df['row'] == self.collector.left_pointer]['actual_linear_vel']).iloc[0]
    #         actual = np.linalg.norm(actual)
    #         commanded = (self.collector._df.loc[self.collector._df['row'] == self.collector.left_pointer]['commanded_linear']).iloc[0]
    #         if(actual < LINEAR_THRESHOLD and commanded[0] > 0):
    #             counter += 0
    #         self.collector.left_pointer += 1
    #     rospy.logerr(f"LINEAR STUCK COUNTER: {counter}")
    #     return counter >= STUCK_THRESHOLD

    # def is_stuck(self):
    #    # make sure timing is ok with pointers
    #     l = len(self.collector._df)
    #     rospy.logerr(f"is_stuck FUNCTION {l}")
    #     if len(self.collector._df) + 1 > DF_THRESHOLD:
    #         self.collector.update_pointers()
    #         if self.check_angular_stuck():
    #             self.collector.rover.stuck = True
    #             return True
    #         if self.check_linear_stuck():
    #             self.collector.rover.stuck = True
    #             return True
    #     self.collector.rover.stuck = False
    #     return False
