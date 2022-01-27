#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pymap3d as pm
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
from lib.planner_utils.cubic_spline_planner import calc_spline_course
from lib.general_utils.read_sd_path import read_sd_map


def cubic(cs, cd): # args에는 1,2,3,4,5,6 등 막 들어 수있음
    sd_path = read_sd_map()
    start_x, start_y = sd_path[cs][cd]
    end_x, end_y = sd_path[cs+20][not cd]
    x=[start_x, end_x]
    y=[start_y, end_y]

    cx, cy, cyaw, ck, s = calc_spline_course(x, y, ds=0.1)
    # plt.scatter(cx,cy)
    # plt.show
    return cx, cy