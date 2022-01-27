from time import time

class Lane_change:
    def __init__(self, ego, d):
        self.ego = ego
        self.d = d
        self.check_time = 0
        self.check = False
        
    def run(self):
        if self.check == False :
            self.check_time = time()
            self.check = True
        
        if time() - self.check_time > 3 and self.check == True :
            self.d = (self.d + 1) % 2
            self.check = False
            
        if self.d == 0 :
            return 0
        else :
            return 1