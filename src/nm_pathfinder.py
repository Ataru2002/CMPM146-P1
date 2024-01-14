import math
import heapq
from queue import PriorityQueue
from queue import Queue

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    start = 0
    path = []
    boxes = {}
    ranges = {}
    detail_points = {}
    source_box = ()
    destination_box = ()
    box_point = {}
    # Extract the graph out of the mesh
    box_graph = {}
    graph = {}
    for looper in mesh.values():
        if start == 1:
            for key in looper.keys():
                box_graph[key] = looper[key]
                if key[0] <= source_point[0] and source_point[0] <= key[1] and key[2] <= source_point[1] and source_point[1] <= key[3]:
                    source_box = key
                if key[0] <= destination_point[0] and destination_point[0] <= key[1] and key[2] <= destination_point[1] and destination_point[1] <= key[3]:
                    destination_box = key
        start += 1

    #debug zone
    #print(detail_points[destination_box])
    #print(destination_box)
    #print(box_graph[destination_box])
    #print(destination_point)
    #print(source_point)
        
    if source_box == () or destination_box == ():
        print("No path!")
        return path, boxes.keys()
    
    range_construction(ranges, box_graph)
    
    box_point[source_point] = source_box
    box_point[destination_point] = destination_box
    path_table = {}
    A_star(detail_points, ranges, path_table, box_graph, box_point, source_box, source_point, destination_box, destination_point)
    starter = destination_point
    test = 0
    
    while starter in path_table and path_table[starter] != -1:
        path.insert(0, starter)
        boxes[box_point[starter]] = box_point[starter]
        starter = path_table[starter]
    
    print(path)
    if starter != source_point:
        print("No path!")
        return path, boxes.keys()
    
    boxes[box_point[starter]] = box_point[starter]
    path.insert(0, starter)
    #path = []
    return path, boxes.keys()

def euclidian_distance(u, v):
    return math.sqrt(((u[0] - v[0]) * (u[0] - v[0])) + (((u[1] - v[1]) * (u[1] - v[1]))))

def A_star(detail_points, ranges, path_table, box_graph, box_point, source_box, source_point, destination_box, destination_point):
    dist = {}
    heap = PriorityQueue()
    for i in detail_points.values():
        dist[i] = math.inf
        path_table[i] = -1
    
    detail_points[source_box] = source_point
    box_point[source_point] = source_box
    heap.put((0, source_box, source_point))
    dist[source_point] = 0

    while not heap.empty():
        current_point = heap.get()

        if current_point[0] != dist[current_point[2]]: 
           continue

        if current_point[1] == destination_box:
            path_table[destination_point] = current_point[2]
            print(destination_point)
            break        

        for i in box_graph[current_point[1]]:
            relationship = (current_point[1], i)
            x_range = ranges[relationship][0]
            y_range = ranges[relationship][1]
            point1 = current_point[2]
            point2 = point_identifier(point1, i, detail_points, box_point, x_range, y_range)
            cost_current = euclidian_distance(point1, point2) + dist[point1] + euclidian_distance(point2, destination_point)
            if point2 not in dist or cost_current < dist[point2]:
                dist[point2] = cost_current
                path_table[point2] = point1
                heap.put((dist[point2], i, detail_points[i]))

def range_construction(ranges, box_graph):
    # construction
    for i in box_graph:
        for j in box_graph[i]:
            current_box = i
            adj_box = j
            x_range = (max(current_box[0], adj_box[0]), min(current_box[1], adj_box[1]))
            y_range = (max(current_box[2], adj_box[2]), min(current_box[3], adj_box[3]))
            ranges[(i, j)] = (x_range, y_range)

def point_identifier(current_point, adj_box, detail_points, box_point, x_range, y_range):
    detail_point = ()

    # horizontal range    
    if y_range[0] == y_range[1] and x_range[0] <= current_point[0] and current_point[0] <= x_range[1]:
        detail_point = (current_point[0], y_range[0])
    elif y_range[0] == y_range[1] and current_point[0] <= x_range[0]:
        detail_point = (x_range[0], y_range[0])
    elif y_range[0] == y_range[1] and current_point[0] >= x_range[1]:
        detail_point = (x_range[1], y_range[0])
    

    # vertical range
    if x_range[0] == x_range[1] and y_range[0] <= current_point[1] and current_point[1] <= y_range[1]:
        detail_point = (x_range[0], current_point[1])
    elif x_range[0] == x_range[1] and current_point[1] <= y_range[0]:
        detail_point = (x_range[0], y_range[0])
    elif x_range[0] == x_range[1] and current_point[1] >= y_range[1]:
        detail_point = (x_range[0], y_range[1])

    detail_points[adj_box] = detail_point
    box_point[detail_point] = adj_box
    current_point = detail_point
    return detail_point