#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pymap3d as pm
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
from lib.cubic_spline_planner import calc_spline_course

base_lat = 37.3851693 #새로운 베이스
base_lon = 126.6562271
base_alt = 15.4

def get_xy(lat, lon, alt): #점들 사이의 새로운 점들을 설정 
    e, n, u = pm.geodetic2enu(lat, lon, alt, base_lat, base_lon, base_alt)
    return e, n

def cubic(name): # args에는 1,2,3,4,5,6 등 막 들어 수있음

    colnames=['lon', 'lat', 'a', 'b', 'c']
    df = pd.read_csv(f'maps/{name}.csv', names=colnames, header=None)
    x=[]
    y=[]
    
    # for i in args: # i=1,2,3,...
    latitude = df.loc[:,'lat'].tolist() # 
    longitude = df.loc[:,'lon'].tolist() # 0 행 부터라서 index 1씩 빼줌
        # output = get_xy(latitude, longitude, 0.5)
        # x.append(output[0])
        # y.append(output[1])

    # cx, cy, cyaw, ck, s = calc_spline_course(x, y, ds=0.1)
    # save_data = list(zip(cx, cy, cyaw, ck, s))

    # save_df = pd.DataFrame(save_data)
    # save_df.to_csv('maps/%s.csv'%name, index=False, header = False)
    # print(f"Map saved to maps/{name}.csv")
    plt.scatter(longitude, latitude)    
    plt.show()

    # return(cx, cy, cyaw, ck, s)


cubic("merged_parking")
#cubic("parking", 1,2,3,4)

