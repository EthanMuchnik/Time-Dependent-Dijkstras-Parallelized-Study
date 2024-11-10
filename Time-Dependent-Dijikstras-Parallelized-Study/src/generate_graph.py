import random
import re

def one_layer_dense(n, file_name):
    with open(file_name, 'w') as file:
        file.write("%%one_layer_dense graph\n")
        file.write(f"{n+2} {n+2} {2*n}\n")
        
        for i in range(1,n+1):
            file.write(f"{0} {i}\n")
        
        for i in range(1,n+1):
            file.write(f"{i} {n+1}\n")


def generate_random_times(og_file_name, new_file_name):
    with open(og_file_name, 'r') as og_file, open(new_file_name, 'w') as new_file:
        line_count = 0
        for line in og_file:
            if line_count < 2:
                new_file.write(line)
            else:
                random_int1 = random.randint(0, 10)
                random_int2 = random.randint(0, 10)
                new_line = line.strip() + f" {random_int1} {random_int2}\n"
                new_file.write(new_line)
            line_count += 1



def generate_random_times_bi_dir(og_file_name, new_file_name):
    with open(og_file_name, 'r') as og_file, open(new_file_name, 'w') as new_file:
        line_count = 0
        for line in og_file:
            if line_count < 2:
                new_file.write(line)
            else:
                numbers = re.findall(r'\d+', line)

                # Convert extracted strings into integers
                numbers = [int(num) for num in numbers]

                random_int1 = random.randint(0, 10)
                random_int2 = random.randint(0, 10)
                new_line = f"{numbers[0] -1} {numbers[1] -1} {random_int1} {random_int2}\n"
                new_file.write(new_line)
                new_line2 = f"{numbers[1] -1} {numbers[0] -1} {random_int1} {random_int2}\n"
                new_file.write(new_line2)
                line_count += 2
            line_count += 1



# def generate_random_from_scratch(n, file_name):

    # must ensure that graph is just one component DAG

    # randomly split n total nodes into levels

    # create source

    # add edge from source to all nodes in the first layer

    # for each layer 2 ... last layer
        # for each node v in the layer
            # for each node u in all previous layers
                # with probability p, add directed edge u -> v
                # if no edge is added, choose random u in previous and add u -> v

    # alternatively we can do this more efficiently as

    # rank nodes from 1 to n
    # for each node i in 2 ... n:
        # sample 
    
    # this ensures that for any vertex in the graph, there exists a path from the source to it


if __name__ == "__main__":    

    # one_layer_dense(8**6, "one_layer_dense.mtx")
    # generate_random_times("networks/road-luxembourg-osm.mtx", "networks/luxembourd-td.mtx")
    # generate_random_times("networks/road-chesapeake.mtx", "networks/chesapeake-td.mtx")
    generate_random_times_bi_dir("networks/road-chesapeake.mtx", "networks/chesapeake-td-bir2.mtx")
