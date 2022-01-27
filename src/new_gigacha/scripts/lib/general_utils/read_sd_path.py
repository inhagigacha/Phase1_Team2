import numpy as np
import csv


def read_sd_map():
    
    sd_map = [[]]
    with open("/home/cvlab/gigacha/planning_ws/src/new_gigacha/scripts/map/sd_simul/x.csv", mode="r") as x_csv_file:
        with open("/home/cvlab/gigacha/planning_ws/src/new_gigacha/scripts/map/sd_simul/y.csv", mode="r") as y_csv_file:
            x_csv_reader = csv.reader(x_csv_file)
            y_csv_reader = csv.reader(y_csv_file)
            # print("1")
            print(f"x_csv_reader : {x_csv_reader}")
            # return
            for s in x_csv_reader:
                for d in range(len(x_csv_reader[0])):
                    sd_map[s][d] = (x_csv_reader[s][d], y_csv_reader[s][d])

    return sd_map

# read_sd_map()