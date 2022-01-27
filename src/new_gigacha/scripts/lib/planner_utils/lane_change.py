from time import time

class Lane_change:
    def __init__(self, ego, d):
        self.ego = ego
        self.ego.lane = d
        self.time_check = 0
        self.check = False

    def run(self):
        if self.check == False:
            self.check = True
            self.time_check = time()

        if time() - self.time_check > 5 and self.check == True :
            self.check = False
            self.ego.lane = (self.ego.lane + 1) % 2
        
        if self.ego.lane == 0 :
            return 0
        else :
            return 1