#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0703

from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import serial
import rospy
import math
import tf
import signal
import sys
from std_msgs.msg import Time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPVT
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randn
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from scipy.linalg import block_diag
import pymap3d as pm
import time as t
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse
import threading
from std_msgs.msg import Int64

from master_node.msg import Serial_Info
# define
magnet_OFF = [0, 0, 0]
PI = 3.141592

# ENU base coordinates
# 송도
# base_lat = 37.383784
# base_lon = 126.654310 
# base_alt = 15.4

#kcity
# base_lat = 37.239231667
# base_lon = 126.773156667
# base_alt = 15.400

# 송도 배달
# 37.3851693
base_lat = 37.3851693
base_lon = 126.6562271
base_alt = 15.4

def signal_handler(sig,frame):
    print('Shut Down')
    sys.exit(0)

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()


        self.e_gps = None
        self.n_gps = None
        self.u_gps = 0
        self.e_gps_pre = 0
        self.n_gps_pre = 0

        self.e_final = None
        self.n_final = None
        self.u_final = 0

        self.e_final_pre = 0
        self.n_final_pre = 0

        self.status = 0

        self.yaw_gps = 0
        self.yaw_imu = 0
        self.yaw_final = None

        # encoder buffer
        self.c = 0
        self.d = 0
        self.e = 0
        self.f = 0


        self.encoder_flag = 0
        self.dis_gps_flag = 0
        self.dis_DR_flag = 0
        self.dis_DR_enc_left = 0
        self.dis_DR_enc_right = 0 
        self.dis_DR_enc = 0
        self.dis_gps_enc = 0
        
        self.displacement = 0
        self.cur_data_left = 0
        self.cur_data_right = 0
        self.displacement_left = 0
        self.displacement_right = 0
        self.diff_left = 0
        self.diff_right = 0
        self.HeadingFrontBackFlg = 1 # 차량 앞뒤 방향 플래그 , offset업데이트시 사용

        self.gps_flag = 0
        self.e_DR = None
        self.n_DR = None
        # self.e_DR = 30.40
        # self.n_DR = 36.72

        self.e_DR = 0
        self.n_DR = 0
        self.position_flag = 0
        self.offset = 0

        self.t_Time_call = 0
        self.t_GPS_call = 0
        self.t_IMU_call = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        self.headingAcc = 99999999
        self.e_filter = None
        self.n_filter = None

        self.cur_point = Point32()
        self.cur_points = PointCloud()
        self.paths = PointCloud()

        self.pub_c = rospy.Publisher('/cur_xy',PointCloud,queue_size = 1)
        self.pub_p = rospy.Publisher('/path',PointCloud,queue_size = 1)
        self.control_input = Serial_Info()
        self.path_steer = 0
        self.speed = 0
        self.gear = None

        self.hAcc = 0

        #plot
        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []
        self.X3 = []
        self.Y3 = []

        self.filter_buffer = []

        self.GPS_ON = 0

        self.flag = False

        rospy.Subscriber('/serial', Serial_Info , self.Get_Dis_left)
        rospy.Subscriber("/Displacement_right", Int64, self.Get_Dis_right)
        rospy.Subscriber('/gps_data/fix',NavSatFix,self.GPS_call)
        rospy.Subscriber('/gps_data/navpvt',NavPVT, self.GPS_Heading)
        rospy.Subscriber('/imu',Imu,self.IMU_call)
        rospy.Subscriber("/control", Serial_Info, self.controlCallback)

        signal.signal(signal.SIGINT,signal_handler)

        while self.flag == False:
            # rospy.Subscriber('/gps_data/fix',NavSatFix,self.GPS_call)
            # rospy.Subscriber("/control", Serial_Info, self.controlCallback)
            # rospy.Subscriber('/imu',Imu,self.IMU_call)

            if(self.e_gps != None):
                self.msg.pose.pose.position.x = float(self.e_gps)
                self.msg.pose.pose.position.y = float(self.n_gps)
            
                self.pub.publish(self.msg)



            if (self.path_steer != 0) and self.yaw_imu != None:
                self.flag = True
                print("ps :",self.path_steer)
                self.offset = (self.path_steer) - self.yaw_imu 
                
                break


        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.flag == False:
                pass
            else:
                #print("posetime :",t.time())
                self.t_Time_call = t.time()

                if self.t_Time_call - self.t_GPS_call > 1.0:
                    self.gps_out = 1
                    #print("DR 주행중")
                else:
                    self.gps_out = 0
                    #print("GPS 주행중")

                if self.t_Time_call - self.t_IMU_call > 0.5:
                    self.imu_out = 1
                else:
                    self.imu_out = 0  
                
                self.decide_heading()
                self.displacement_filter()
                self.DR()
                self.decide_position()
                
                self.msg_write(self.msg)
                #print(self.HeadingFrontBackFlg)
                #print("gps law:: E =",self.e_gps,"  N =",self.n_gps)
                print("좌표 ::  E =",round(self.e_final,2),"   N =",round(self.n_final,2))
                #print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  Heading:",round(self.yaw_final))
                self.pub.publish(self.msg)

                rate.sleep()
            # while self.flag == False:
            #     # rospy.Subscriber('/gps_data/fix',NavSatFix,self.GPS_call)
            #     # rospy.Subscriber("/control", Serial_Info, self.controlCallback)
            #     # rospy.Subscriber('/imu',Imu,self.IMU_call)

            #     if(self.e_gps != None):
            #         self.msg.pose.pose.position.x = float(self.e_gps)
            #         self.msg.pose.pose.position.y = float(self.n_gps)
                
            #         self.pub.publish(self.msg)



            #     if (self.path_steer != 0) and self.yaw_imu != None:
            #         self.flag = True
            #         print("ps :",self.path_steer)
            #         self.offset = (self.path_steer) - self.yaw_imu 
                    
            #         break

            

    def gps_check(self): #gps신뢰도 판단하는 부분 
        if self.hAcc < 150:
            self.position_flag = 0
        else:
            self.position_flag = 1

    def msg_write(self,msg):
        self.msg.pose.pose.position.x = float(self.e_final)
        self.msg.pose.pose.position.y = float(self.n_final)
        self.msg.pose.pose.position.z = float(self.velocity)
        self.msg.twist.twist.angular.z = self.yaw_final
        self.e_final_pre = self.e_final
        self.n_final_pre = self.n_final

    def controlCallback(self,data):
        self.control_input = data
        self.path_steer = data.path_steer

    def GPS_call(self,data):
        #print("calltime :",data.header.stamp)
        self.t_GPS_call = t.time()
        
        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.status = data.status.status #RTK status
        

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)
        #print("gps law:: E =",self.e_gps,"  N =",self.n_gps)
        #print("gps lat:: lat =",lat,"  lon =",lon)

        #gps 신뢰도 판단
        # self.gps_check()
        # self.e_gps_pre = self.e_gps
        # self.n_gps_pre = self.n_gps

        if self.GPS_ON == 0:
            self.GPS_ON = 1
            self.e_DR = self.e_gps
            self.n_DR = self.n_gps


    def GPS_Heading(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360
        self.hAcc = data.hAcc
        self.headingAcc =data.headAcc
        
    def IMU_call(self,data):
        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360
        

    #serial callback  hz : very fast
    #왼쪽 엔코더 콜백함수 
    def Get_Dis_left(self, msg):
        self.cur_data_left = msg.encoder
        self.speed = msg.speed
        self.gear = msg.gear

    # 오른쪽 엔코더 콜백함수
    def Get_Dis_right(self, data):
        res = data.data
        #print("R_encoder : ",res)
        self.cur_data_right = int(res)

    # 양쪽 엔코더
    def displacement_filter(self):
        # self.displacement_right = self.cur_data_right
        if self.encoder_flag == 0:
            self.displacement_left = self.cur_data_left
            self.displacement_right = self.cur_data_right
            self.encoder_flag = 1

        if (abs(self.cur_data_left - self.displacement_left) > 100):
            self.displacement_left = self.displacement_left + self.diff_left
        else:
            self.diff_left = self.cur_data_left - self.displacement_left
            self.displacement_left = self.cur_data_left

        
        if (abs(self.cur_data_right - self.displacement_right) > 100):
            self.displacement_right = self.displacement_right + self.diff_right
        else:
            self.diff_right = self.cur_data_right - self.displacement_right            # 여기서 displacement_right는 이전값을 의미한다
            self.displacement_right = self.cur_data_right                              # 여기서 displacement_right는 현재값을 의미한다


    #Dead Reckoning 
    # def DR(self):
    #     if (self.dis_DR_flag == 0) and (self.displacement_left !=0):
    #         self.c = self.displacement_left
    #         self.d = self.displacement_left
    #         self.dis_DR_flag = 1
    #         self.e_DR = self.e_gps
    #         self.n_DR = self.n_gps
  
    #     elif self.dis_DR_flag == 1 :
    #         self.c = self.d
    #         self.d = self.displacement_left 

    #     if ((self.d - self.c) < -10000000):
    #         self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100 # encoder pulse를 m단위로 치환
    #     else:
    #         self.dis_DR_enc_left = (self.d - self.c)*1.6564/100

    #     if ((self.d - self.c)>0):
    #         self.HeadingFrontBackFlg = 1               
    #     else:
    #         self.HeadingFrontBackFlg = 0
            
    #     self.dis_DR_enc = (self.dis_DR_enc_left + self.dis_DR_enc_right) / 2 # encoder 왼쪽 오른쪽 평균내기

    #     self.velocity = self.dis_DR_enc

    #     self.e_DR += self.dis_DR_enc*math.cos(math.radians(self.yaw_final))
    #     self.n_DR += self.dis_DR_enc*math.sin(math.radians(self.yaw_final))


    # Dead Reckoning 양쪽 엔코더
    def DR(self):
        if (self.dis_DR_flag == 0) and (self.displacement_left !=0):
            self.c = self.displacement_left
            self.d = self.displacement_left
            self.e = self.displacement_right
            self.f = self.displacement_right
            self.dis_DR_flag = 1
        
        elif self.dis_DR_flag == 1:
            self.c = self.d
            self.d = self.displacement_left 

            self.e = self.f
            self.f = self.displacement_right         

        if ((self.d - self.c) < -10000000):
            self.dis_DR_enc_left = (self.d + (256**4 - self.c))/60.852 # encoder pulse를 m단위로 치환
        else:
            self.dis_DR_enc_left = (self.d - self.c)/60.852 # 1.6564/100

        if ((self.f - self.e) < -10000000):
            self.dis_DR_enc_right = (self.f + (256**4 - self.e))/60.852
        else:
            self.dis_DR_enc_right = (self.f - self.e)/60.852
        
        #print("차이 :",(self.f - self.e)," flg :",self.HeadingFrontBackFlg)
        if ((self.f - self.e)<0):
            self.HeadingFrontBackFlg = 0               
        elif ((self.f - self.e)>0) :
            self.HeadingFrontBackFlg = 1

        self.dis_DR_enc = (self.dis_DR_enc_right + self.dis_DR_enc_left)/2 # encoder 왼쪽 오른쪽 평균내기

        self.velocity = self.dis_DR_enc/0.1

        self.e_DR += self.dis_DR_enc*math.cos(math.radians(self.yaw_final))
        self.n_DR += self.dis_DR_enc*math.sin(math.radians(self.yaw_final))

    def yaw_check(self): # 차량이 똑바로 직진시 (steering 받기), heading 정확도가 200000보다 낮을 시 heading값 보정
        # if abs(int(self.control_input.steer)) < 10 and self.HeadingFrontBackFlg == 1 and self.headingAcc < 1000000:
            # if self.headingAcc < 1000000:
        if self.HeadingFrontBackFlg == 1 and self.headingAcc < 700000 and self.gear == 0 :
            self.yaw_flag = 1               
        else:
            self.yaw_flag = 0
        # print("엔코더 실시간 차이",(self.dis_DR_enc_right - self.dis_DR_enc_left))

    def decide_heading(self):
        self.yaw_check()
        if self.imu_out == 0:               # imu 정상
            if self.gps_out == 0:           # gps 정상
                if self.yaw_flag == 1: # gps o,imu o & 신뢰도 o & 앞으로 갈 때
                    self.offset = self.yaw_gps - self.yaw_imu 
                self.yaw_final = self.yaw_imu + self.offset
            else:
                self.yaw_final = self.yaw_imu + self.offset                  # gps가 꺼졌을 때
        else:
            if self.gps_out == 0:                                            # imu가 꺼졌을 떄   
                self.yaw_final = self.yaw_gps
                # print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  f:",round(self.yaw_final),"   gt",self.t_GPS_call,"  go",self.gps_out)                                                          # imu / gps가 꺼졌을 때
            else:  
                self.yaw_final = self.path_steer - 90 # 이상태가 오면 그냥 대회 포기하면 됩니다.


        self.yaw_final = self.yaw_final % 360 

    def decide_position(self):
        if self.gps_out == 0:                                               # gps o
            self.e_final = self.e_gps
            self.n_final = self.n_gps
        
            self.e_DR = self.e_final
            self.n_DR = self.n_final
            #print("gps 주행 중")      
                
        else:                                                               # gps x
            self.e_final = self.e_DR
            self.n_final = self.n_DR
            #print("dead Reckoning 주행 중")


    def ps(self,X1,Y1,X2,Y2,X3,Y3):
        plt.ion()
        animated_plot = plt.plot(X1, Y1, 'r')[0]
        animated_plot2 = plt.plot(X2, Y2, 'b')[0]
        # animated_plot3 = plt.plot(X3, Y3, 'g')[0]
        ps1=1

        # for i in range(0, len(X1)):
        #     # if i>30:
        #     animated_plot.set_xdata(X1[0:i])
        #     animated_plot.set_ydata(Y1[0:i])
        #     animated_plot2.set_xdata(X2[0:i])
        #     animated_plot2.set_ydata(Y2[0:i])
            # animated_plot3.set_xdata(X3[0:i])
            # animated_plot3.set_xdata(Y3[0:i])

            # else:
                # animated_plot.set_xdata(X1[0:i])
                # animated_plot.set_ydata(Y1[0:i])
                # animated_plot2.set_xdata(X2[0:i])
                # animated_plot2.set_ydata(Y2[0:i])
        plt.draw()
        plt.pause(0.01)






if __name__ == "__main__":
    loc = Localization()
