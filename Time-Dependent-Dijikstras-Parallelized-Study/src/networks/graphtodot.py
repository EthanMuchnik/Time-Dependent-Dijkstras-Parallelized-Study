graph_file = "road-luxembourg-osm.mtx"
dot_file = "luxembourg-dot.txt"

with open(graph_file, 'r') as graph, open(dot_file, 'w') as dot:
    count = 0
    for line in graph:
        if count > 2:
            verts = line.strip().split()
            u = verts[0]
            v = verts[1]
            dot.write(f"{u} -> {v};\n")

        count += 1