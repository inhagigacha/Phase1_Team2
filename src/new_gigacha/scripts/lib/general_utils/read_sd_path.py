
import numpy as np
import csv


def read_sd_map():
    
    sd_map = [[]]
    with open("map/x.csv", mode="r") as x_csv_file:
        with open("map/y.csv", mode="r") as y_csv_file:
            x_csv_reader = csv.reader(x_csv_file)
            y_csv_reader = csv.reader(y_csv_file)
            
            for s in x_csv_reader:
                for d in range(x_csv_reader[0]):
                    sd_map[s][d] = (x_csv_reader[s][d], y_csv_reader[s][d])


    return sd_map