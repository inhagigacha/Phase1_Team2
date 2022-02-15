import csv

def combine_csv(number_of_files):
    
    real_array = []
    float_array = []

    for i in range(1, number_of_files):
        with open(f'/home/gigacha/Phase1_Team2/src/new_gigacha/src/new_gigacha/scripts/lib/new_mapping/maps/{i}_map.csv', mode="r", newline = '') as csv_file:
            for line in csv_file.readlines():
                array = line.split(',')
                float_array = [float(array[0]), float(array[1])]
                real_array.append(float_array)
            
    with open('/home/gigacha/Phase1_Team2/src/new_gigacha/src/new_gigacha/scripts/lib/new_mapping/maps/all_nodes.csv', 'w', newline = '') as all_nodes_file: 
        writer = csv.writer(all_nodes_file)
        writer.writerows(real_array)

combine_csv(10)