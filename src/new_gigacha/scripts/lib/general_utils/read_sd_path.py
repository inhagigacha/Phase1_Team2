import numpy as np
import csv


def read_sd_map():
    
    sd_map = [[]]
    with open("/home/cvlab/gigacha/planning_ws/src/new_gigacha/scripts/map/sd_simul/x.csv", mode="r") as x_csv_file:
        with open("/home/cvlab/gigacha/planning_ws/src/new_gigacha/scripts/map/sd_simul/y.csv", mode="r") as y_csv_file:
            x_csv_reader = csv.reader(x_csv_file)
            y_csv_reader = csv.reader(y_csv_file)
            x = list(x_csv_reader)
            y = list(y_csv_reader)
            for s in range(len(x)):
                for d in range(len(x[1])):
                    sd_map.append([float(x[s][d]), float(y[s][d])])
                    print("check")

    return sd_map

# read_sd_map()