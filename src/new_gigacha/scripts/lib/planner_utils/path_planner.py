from lib.planner_utils.lane_change import Lane_change
from lib.planner_utils.csv_curve_latlon_0127 import cubic
import rospy

class PathPlanner:
    def __init__(self, ego):
        self.ego = ego
        self.d = self.ego.lane

    def run(self):
        if self.d != self.ego.lane:
            self.ego.local_path = cubic(self.ego.index, self.ego.lane)