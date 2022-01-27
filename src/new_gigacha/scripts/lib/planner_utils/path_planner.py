from lib.planner_utils.lane_change import Lane_change
from lib.planner_utils.csv_curve_latlon_0127 import cubic
import rospy

class PathPlanner:
    def __init__(self, ego):
        self.ego = ego
        self.d = 1

    def run(self):
        cc = Lane_change(self.ego, self.d)
        d = cc.run()
        if d != self.d:
            self.ego.local_path = cubic(self.ego.index, self.d)
        self.d = d