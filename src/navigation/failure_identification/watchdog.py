import numpy as np
import rospy
from pandas import DataFrame

# 10 or more data objects flagged as stuck, then you are actually stuck
DF_THRESHOLD = 100
STUCK_THRESHOLD = 50
ANGULAR_THRESHOLD = 0.001
LINEAR_THRESHOLD = 0.55


class WatchDog:
    def __init__(self, collector_in):
        self.collector = collector_in
        

    # def is_stuck(self, dataframe: DataFrame):
    #     return False
    def update_pointers(self):
       self.collector.right_pointer = self.collector.row_counter
       self.collector.left_pointer = self.collector.right_pointer - DF_THRESHOLD
       rospy.logerr(f"RIGHT: {self.collector.right_pointer} LEFT: {self.collector.left_pointer}")

    def check_angular_stuck(self, dataframe: DataFrame):
        counter = 0
        self.collector.left_pointer = self.collector.right_pointer - DF_THRESHOLD
        while (self.collector.left_pointer < self.collector.right_pointer):
            actual = (dataframe.loc[self.collector._df['row'] == self.collector.left_pointer]['angular_velocity']).iloc[0]
            commanded = (dataframe.loc[self.collector._df['row'] == self.collector.left_pointer]['cmd_vel_twist']).iloc[0]
            # rospy.logerr(f"ANGULAR ACTUAL: {actual} COMMANDED: {commanded}")
            if(actual < ANGULAR_THRESHOLD and commanded > 0):
                counter += 1
            self.collector.left_pointer += 1
        # rospy.logerr(f"ANGULAR STUCK COUNTER: {counter}")
        return counter >= STUCK_THRESHOLD

    def check_linear_stuck(self, dataframe: DataFrame):
        counter = 0
        self.collector.left_pointer = self.collector.right_pointer - DF_THRESHOLD
        while (self.collector.left_pointer < self.collector.right_pointer):
            actual = (dataframe.loc[self.collector._df['row'] == self.collector.left_pointer]['linear_velocity']).iloc[0]
            # actual = np.linalg.norm(actual)
            commanded = (dataframe.loc[self.collector._df['row'] == self.collector.left_pointer]['cmd_vel_x']).iloc[0]
            rospy.logerr(f"LINEAR ACTUAL: {actual} COMMANDED: {commanded} ")
            if(actual < LINEAR_THRESHOLD and commanded > 0):
                counter += 1
            self.collector.left_pointer += 1
        rospy.logerr(f"LINEAR STUCK COUNTER: {counter}")
        return counter >= STUCK_THRESHOLD

    def is_stuck(self, dataframe: DataFrame):
       # make sure timing is ok with pointers
        l = len(dataframe)
        # rospy.logerr(f"is_stuck FUNCTION LEN {l} RIGHT {self.collector.right_pointer}")
        if len(dataframe) > DF_THRESHOLD:
            self.update_pointers()
            if self.check_angular_stuck(dataframe):
                # self.collector.rover.stuck = True
                return True
            if self.check_linear_stuck(dataframe):
                # self.collector.rover.stuck = True
                return True
        # self.collector.rover.stuck = False
        return False

