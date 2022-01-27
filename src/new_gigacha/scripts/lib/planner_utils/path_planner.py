from Phase1_Team2.src.new_gigacha.scripts.lib.planner_utils.lane_change import Lane_change
import rospy
from lib.new_mapping.csv_curve_latlon_0127 import cubic

class PathPlanner:
    def __init__(self, ego):
        self.ego = ego
        self.d = 1
    def run(self):
        
        d = Lane_change(self.ego, self.d)
        if d != self.d:
            self.ego.local_path = cubic(self.ego.index, self.d)
        self.d = d