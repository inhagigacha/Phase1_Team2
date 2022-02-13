from math import hypot
from time import time

class MissionPlanner:
    def __init__(self, ego):
        self.ego = ego
        self.pre_time = -1
        self.check = 0
        self.aft_time = -1
        
    def mission2(self, index):
        

    def run(self):
        self.ego.mode = "mission0"
        # mission 1
        dist1 = hypot(self.ego.pose.x - self.ego.global_path.x[990], self.ego.pose.y - self.ego.global_path.y[990]) 
        
        if dist1 < 1 and self.ego.mode == "mission0" and self.pre_time < 0 and self.check == 0 :
            self.ego.status = "mission1"
            self.ego.mode = "stop"
            self.pre_time = time()
        if dist1 < 1.5 and self.ego.mode == "stop" and time() - self.pre_time > 5:
            self.ego.mode = "driving"
            self.pre_time = -1
            self.check = 1

        # mission 2
        dist2 = hypot(self.ego.pose.x - self.ego.global_path.x[1113], \
                    self.ego.pose.y - self.ego.global_path.y[1113]  )
        # print(f"dist2: {dist2}")
        
        # 서기 전 준비
        if dist2 < 1.0 and self.ego.mode == "parking_driving" and self.aft_time < 0:
            self.ego.status = "parking complete"
            self.ego.mode = "emergency_stop"
            self.aft_time = time()
        
        # 가는거
        if self.ego.status == "parking complete" and time() - self.aft_time > 3:
            self.ego.status = "parking backward"
            self.ego.mode = "backward"
            self.aft_time = -1
            
        # mission 3
        dist3 = hypot(self.ego.pose.x - self.ego.global_path.x[1225], \
                    self.ego.pose.y - self.ego.global_path.y[1225]  )
        
        if dist3 < 2.0 and self.ego.status == "parking backward" and self.aft_time < 0 :
            self.ego.status = "parking end"
            self.ego.mode = "emergency_stop"
            self.aft_time = time()
        # print(f"dist3: {dist3}")
        if self.ego.status == "parking end" and time() - self.aft_time > 3:
            self.ego.status = "general"
            self.ego.mode = "driving"
            self.ego.mission = "gogo"
            
        # mission 4
        dist4 = hypot(self.ego.pose.x - self.ego.global_path.x[1225], self.ego.pose.y - self.ego.global_path.y[1225])
        if dist4 < 1 and self.ego.status == "mission3" and self.aft_time < 0:
            self.ego.status = "mission4"
            self.ego.mode = "driving"
            self.ego.
        # mission 5