import csv

def combine_csv(number_of_files):
    
    real_array = [[]]

    for i in range(1, number_of_files):
        with open(f'/home/gigacha/Phase1_Team2/src/new_gigacha/scripts/lib/new_mapping/maps/{i}_map.csv', mode="r") as csv_file:
            for line in csv_file.readlines():
                array = line.split(',')
                real_array.append(array)
            
    with open('/home/gigacha/Phase1_Team2/src/new_gigacha/scripts/lib/new_mapping/maps/all_nodes.csv', 'w') as all_nodes_file: 
        writer = csv.writer(all_nodes_file)
        writer.writerows(real_array)
        writer.close()

combine_csv(8)
