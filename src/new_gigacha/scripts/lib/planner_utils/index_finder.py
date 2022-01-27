from math import hypot
from lib.planner_utils.lane_change import Lane_change
from lib.general_utils.read_sd_path import read_sd_map
class IndexFinder:
    def __init__(self, ego):
        self.ego = ego
        self.save_idx = 0
        self.d = 1

    def run(self):
        min_dis = -1
        min_idx = 0
        
        step_size = 100

        self.d = Lane_change(self.ego, self.d)
        for i in range(max(self.ego.index - step_size, 0), self.ego.index + step_size):
            try:
                dis = hypot(self.ego.global_path.x[i][self.d] - self.ego.pose.x, self.ego.global_path.y[i][self.d] - self.ego.pose.y)
            except IndexError:
                break
            if (min_dis > dis or min_dis == -1) and self.save_idx <= i:
                min_dis = dis
                min_idx = i
                self.save_idx = i
        # print(f'lateral_error : {min_dis}')
        print(f'{min_dis}')
   
        self.ego.index = min_idx
        return self.ego.index, self.d