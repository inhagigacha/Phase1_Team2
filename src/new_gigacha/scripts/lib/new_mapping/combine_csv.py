import csv

def combine_csv(number_of_files):
    
    real_array = []

    for i in range(1, number_of_files):
        with open(f'/maps/{i}_map.csv', mode="r", newline = '') as csv_file:
            for line in csv_file.readlines():
                array = line.split(',')
                real_array.append(array)
            
    with open('/maps/all_nodes.csv', 'wb', newline = '') as all_nodes_file: 
        writer = csv.writer(all_nodes_file)
        writer.writerows(real_array)

combine_csv(9)