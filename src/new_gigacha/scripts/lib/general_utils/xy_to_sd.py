import csv

def xy_to_sd():

  with open ('lane1.csv', mode="r") as left_lane_file:
    left_lane_x_reader = csv.reader(left_lane_file[0])
    left_lane_y_reader = csv.reader(left_lane_file[1])


  with open ('lane2.csv', mode="r") as right_lane_file:
    right_lane_x_reader = csv.reader(right_lane_file[0])
    right_lane_y_reader = csv.reader(right_lane_file[1]) # 이 친구들 상대경로로 바꿔주어야 함니다...!
      
  llx = list(left_lane_x_reader)
  lly = list(left_lane_y_reader)
  rlx = list(right_lane_x_reader)
  rly = list(right_lane_y_reader)

  x_writer = open("../../map/sd_simul/x.csv", mode="w", newline='')
  y_writer = open("../../map/sd_simul/y.csv", mode="w", newline='')
  
  for s in range(len(llx)):
    x_writer.writerow([llx[s], rlx[s]])
    y_writer.writerow([lly[s], rly[s]])
  
  x_writer.close()
  y_writer.close()

xy_to_sd()