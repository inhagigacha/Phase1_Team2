<<<<<<< Updated upstream
import csv

def gps_to_sd():

  with open ('lane1.csv', mode="r") as left_lane_file:
    with open ('lane2.csv', mode="r") as right_lane_file:
      left_lane_x_reader = csv.reader(left_lane_file[0])
      left_lane_y_reader = csv.reader(left_lane_file[1])
      right_lane_x_reader = csv.reader(right_lane_file[0])
      right_lane_y_reader = csv.reader(right_lane_file[1])

  with open ('x.csv', mode="w", newline='') as x_writer:
    with open ('y.csv', mode="w", newline='') as y_writer:
      for s in range(left_lane_x_reader):
        x_writer.writerow([left_lane_x_reader[s], right_lane_x_reader[s]])
        y_writer.writerow([left_lane_y_reader[s], right_lane_y_reader[s]])

if __name__=="main":
  gps_to_sd()
=======
import pandas as pd
d1 = pd.read_csv('C:/Users/ss/Desktop/maps/lane1.csv', header=None,delimiter=" ")
d2 = pd.read_csv('C:/Users/ss/Desktop/map2/x.csv', header=None,delimiter=" ")
d2.iloc[:, [0]]=d1[0]
>>>>>>> Stashed changes
