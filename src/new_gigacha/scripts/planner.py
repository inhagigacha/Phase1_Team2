from lib.general_utils.sig_int_handler import Activate_Signal_Interrupt_Handler
from lib.planner_utils.index_finder import IndexFinder
from lib.general_utils.ego import EgoVehicle
from lib.general_utils.read_sd_path import read_sd_map
from lib.planner_utils.mission_planner import MissionPlanner
from lib.planner_utils.path_planner import PathPlanner
from lib.planner_utils.sensor_hub import SensorHub
from new_gigacha.msg import Planning_Info
from lib.planner_utils.lane_change import Lane_change
from lib.planner_utils.mission1 import Mission1

import rospy

class Planner:
    def __init__(self):
        rospy.init_node("Planner", anonymous=False)
        self.ego = EgoVehicle()
        self.ego.global_path.x, self.ego.global_path.y = read_sd_map() ##

        self.sensor_hub = SensorHub(self.ego)
        self.whereami = IndexFinder(self.ego)
        self.path_planner = PathPlanner(self.ego) ## 
        # self.mission_planner = MissionPlanner(self.ego)
        self.planning_pub = rospy.Publisher("/planner", Planning_Info, queue_size=1)
        self.planning_msg = Planning_Info()
        self.lanechange = Lane_change(self.ego, self.ego.lane)
        
        self.mission_1 = Mission1()

    def publish_planning_info(self):
        self.planning_msg.index = self.ego.index
        self.planning_msg.lane = self.ego.lane
        self.planning_msg.mode = self.ego.mode
        self.planning_msg.local = self.ego.pose
        self.planning_pub.publish(self.planning_msg)

    def run(self):
        self.whereami.run() # local
        self.mission_planner.run()
        self.publish_planning_info()

if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    pp = Planner()
    rate = rospy.Rate(20)
    while True:
        pp.run()
        rate.sleep()