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
    box_graph = {}
    for looper in mesh.values():
        if start == 1:
            for key in looper.keys():
                box_graph[key] = looper[key]
                if key[0] <= source_point[0] and source_point[0] <= key[1] and key[2] <= source_point[1] and source_point[1] <= key[3]:
                    source_box = key
                if key[0] <= destination_point[0] and destination_point[0] <= key[1] and key[2] <= destination_point[1] and destination_point[1] <= key[3]:
                    destination_box = key
        start += 1
        
    if source_box == () or destination_box == ():
        print("No path!")
        return path, boxes.keys()
    
    range_construction(ranges, box_graph)
    
    box_point[source_point] = source_box
    box_point[destination_point] = destination_box
    path_tablef = {}
    path_tableb = {}
    meetup = A_star(detail_points, ranges, path_tablef, path_tableb, box_graph, box_point, source_box, source_point, destination_box, destination_point)

    starter = meetup
    while starter in path_tablef and path_tablef[starter] != -1:
        path.insert(0, starter)
        boxes[box_point[starter]] = box_point[starter]
        starter = path_tablef[starter]

    if starter != source_point:
        print("No path!")
        return path, boxes.keys()
    path.insert(0, starter)
    boxes[box_point[starter]] = box_point[starter]

    
    
    starter = meetup
    while starter in path_tableb and path_tableb[starter] != -1:
        path.append(starter)
        boxes[box_point[starter]] = box_point[starter]
        starter = path_tableb[starter]
    if starter != destination_point:
        print("No path!")
        return path, boxes.keys()
    path.append(starter)
    boxes[box_point[starter]] = box_point[starter]

    

    return path, boxes.keys()

def euclidian_distance(u, v):
    return math.sqrt(((u[0] - v[0]) * (u[0] - v[0])) + (((u[1] - v[1]) * (u[1] - v[1]))))

def A_star(detail_points, ranges, path_tablef, path_tableb, box_graph, box_point, source_box, source_point, destination_box, destination_point):
    distf = {}
    distb = {}
    heap = PriorityQueue()
    
    detail_points[source_box] = source_point
    box_point[source_point] = source_box
    heap.put((0, source_box, source_point, destination_box))
    heap.put((0, destination_box, destination_point, source_box))
    distf[source_point] = 0
    distb[destination_point] = 0

    print(source_point)
    print(destination_point)

    while not heap.empty():
        current_instance = heap.get()
        
        current_dist = current_instance[0]
        current_box = current_instance[1]
        current_point = current_instance[2]
        current_destination = current_instance[3]

        if current_destination == destination_box:
            if current_point in distf and current_dist != distf[current_point]: 
                continue
        if current_destination == source_box:
            if current_point in distb and current_dist != distb[current_point]:
                continue   
        
        
        if current_destination == destination_box:
            if current_point in path_tableb:
                return current_point
        if current_destination == source_box:
            if current_point in path_tablef:
                return current_point

        for i in box_graph[current_box]:
            relationship = (current_box, i)
            x_range = ranges[relationship][0]
            y_range = ranges[relationship][1]
            point1 = current_point
            point2 = point_identifier(point1, i, detail_points, box_point, x_range, y_range)
            cost_current = euclidian_distance(point1, point2)

            if current_destination == destination_box:
                cost_current += distf[point1] + euclidian_distance(point2, destination_point)
            else:
                cost_current += distb[point1] + euclidian_distance(point2, source_point)

            if current_destination == source_box:
                if point2 not in distb or cost_current < distb[point2]:
                    distb[point2] = cost_current
                    path_tableb[point2] = point1
                    heap.put((distb[point2], i, detail_points[i], current_destination))

            if current_destination == destination_box:
                if point2 not in distf or cost_current < distf[point2]:
                    distf[point2] = cost_current
                    path_tablef[point2] = point1
                    heap.put((distf[point2], i, detail_points[i], current_destination))
            

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