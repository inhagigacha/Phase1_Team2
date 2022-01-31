import numpy as np
import csv


def read_sd_map():
    
    sd_map_x = []
    sd_map_y = []
    with open("../../map/sd_simul/x.csv", mode="r") as x_csv_file:
        with open("../../map/sd_simul/y.csv", mode="r") as y_csv_file:
            x_csv_reader = csv.reader(x_csv_file)
            y_csv_reader = csv.reader(y_csv_file)
            x = list(x_csv_reader)
            y = list(y_csv_reader)
            for s in range(len(x)):
                sd_map_x.append([float(x[s][0]), float(x[s][1])])
                sd_map_y.append([float(y[s][0]), float(y[s][1])])
    return sd_map_x, sd_map_y