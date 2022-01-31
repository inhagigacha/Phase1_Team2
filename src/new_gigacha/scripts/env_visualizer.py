import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from new_gigacha.msg import Local
from lib.general_utils.sig_int_handler import Activate_Signal_Interrupt_Handler
from lib.general_utils.read_global_path import read_global_path
from lib.general_utils.read_sd_path import read_sd_map
from lib.controller_utils.state import State

class environmentVisualizer:
    def __init__(self):
        rospy.init_node("Environment_Visualizer", anonymous=False)
        self.vis_global_path_pub = rospy.Publisher("/vis_global_path", PointCloud, queue_size=1)
        self.vis_global_path_msg1 = PointCloud()
        self.vis_global_path_msg1.header.frame_id = "map"
        self.vis_global_path_msg2 = PointCloud()
        self.vis_global_path_msg2.header.frame_id = "map"
        self.vis_trajectory = PointCloud()
        self.vis_trajectory.header.frame_id = "map"


        self.vis_trajectory_pub = rospy.Publisher("/vis_trajectory", PointCloud, queue_size=1)

        # global_path_= read_global_path('songdo', 'parking_simul')
        global_path_x, global_path_y = read_sd_map()

        for i in range(len(global_path_x)):
            self.vis_global_path_msg1.points.append(Point32(global_path_x[i][0], global_path_y[i][0], 0))
            self.vis_global_path_msg1.points.append(Point32(global_path_x[i][1], global_path_y[i][1], 0))

        # for i in range(len(global_path_x)):
        #     self.vis_global_path_msg2.points.append(Point32(global_path_x[i][1], global_path_y[i][1], 0))


        self.vis_pose_pub = rospy.Publisher("/vis_pose", Odometry, queue_size=1)
        self.vis_pose_msg = Odometry()
        self.vis_pose_msg.header.frame_id = "map"
        

        rospy.Subscriber('/pose', Local, self.poseCallback)

        # self.obsmap_pub = rospy.Publisher("/vis_map", PointCloud, queue_size=1)
        # self.obsmap = PointCloud()
        # self.obsmap.header.frame_id = "map"
    

        # self.local_path_pub = rospy.Publisher("/vis_local_path", PointCloud, queue_size=1)
        # self.obs_pub = rospy.Publisher("/vis_obs_pub", PointCloud, queue_size=1)
        # self.target_pub = rospy.Publisher("/vis_target", PointCloud, queue_size=1)

        # self.vis_local_path = PointCloud()
        # self.vis_local_path.header.frame_id = "map"


        # self.obs = PointCloud()
        # self.obs.header.frame_id = "map"

        # self.target = PointCloud()
        # self.target.header.frame_id = "map"
    def poseCallback(self, msg):

        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        x = msg.x
        y = msg.y
        self.vis_pose_msg.pose.pose.position.x = x
        self.vis_pose_msg.pose.pose.position.y = y
        self.vis_trajectory.header.stamp = rospy.Time.now()
        
        self.vis_trajectory.points.append(Point32(x, y, 0))



    def run(self):
        print(f"Publishing maps for visualization")
        self.vis_global_path_msg1.header.stamp = rospy.Time.now()
        self.vis_global_path_pub.publish(self.vis_global_path_msg1)
        # self.vis_global_path_msg2.header.stamp = rospy.Time.now()
        # self.vis_global_path_pub.publish(self.vis_global_path_msg2)


        self.vis_pose_msg.header.stamp = rospy.Time.now()
        self.vis_pose_pub.publish(self.vis_pose_msg)
        self.vis_trajectory_pub.publish(self.vis_trajectory)
        # self.obsmap.points = self.ego.obs_map.points
        # self.obsmap.header.stamp = rospy.Time.now()
        # self.obsmap_pub.publish(self.obsmap)



if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    vv = environmentVisualizer()
    rate = rospy.Rate(10)
    while True:
        vv.run()
        rate.sleep()
