import math
import heapq

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
    if source_box == () or destination_box == ():
        print("No path!")
        return path, boxes.keys()
    point_construction(detail_points, box_point, box_graph, source_box, source_point)
    path_table = {}
    A_star(detail_points, path_table, box_graph, source_box, source_point, destination_box, destination_point)
    starter = destination_point
    test = 0
    box_point[source_point] = source_box
    box_point[destination_point] = destination_box
    while starter in path_table and path_table[starter] != -1:
        path.insert(0, starter)
        boxes[box_point[starter]] = box_point[starter]
        starter = path_table[starter]
    
    if starter != source_point:
        print("No path!")
        return path, boxes.keys()
    
    boxes[box_point[starter]] = box_point[starter]
    path.insert(0, starter)
    return path, boxes.keys()

# tmr finish up dijkstra and A*

def euclidian_distance(u, v):
    return math.sqrt(((u[0] - v[0]) * (u[0] - v[0])) + (((u[1] - v[1]) * (u[1] - v[1]))))

def A_star(detail_points, path_table, box_graph, source_box, source_point, destination_box, destination_point):
    dist = {}
    heap = []

    for i in detail_points.values():
        dist[i] = 1000000000
        path_table[i] = -1
    heapq.heapify(heap)
    heapq.heappush(heap, (0, source_box, source_point))
    dist[source_point] = 0

    while len(heap) > 0:
        current_point = heap[0]
        heapq.heappop(heap)
        
        if current_point[1] == destination_box:
            path_table[destination_point] = current_point[2]
            break

        if current_point[0] != dist[current_point[2]]: 
           continue

        for i in box_graph[current_point[1]]:
            point1 = current_point[2]
            point2 = detail_points[i]
            cost_current = euclidian_distance(point1, point2) + dist[point1] + euclidian_distance(point2, destination_point)
            if point2 not in dist or cost_current < dist[point2]:
                dist[point2] = cost_current
                path_table[point2] = point1
                heapq.heappush(heap, (dist[point2], i, detail_points[i]))

def point_construction(detail_points, box_point, box_graph, source_box, source_point):
    # construction
    # can be slow so keep this in mind
    queue = []
    
    visited = {}
    for i in box_graph.keys():
        visited[i] = False

    queue.append(source_box)
    detail_points[source_box] = source_point
    visited[source_box] = True

    while len(queue) > 0: 
        current_box = queue[0]
        current_point = detail_points[current_box]
        queue.pop(0) # takes O(n) so keep this in mind
        for adj_box in box_graph[current_box]:
            # for each box determine the detail points from the current_box

            if visited[adj_box] == False:
                detail_point = ()

                visited[adj_box] = True
                y_range = (max(current_box[2], adj_box[2]), min(current_box[3], adj_box[3]))
                x_range = (max(current_box[0], adj_box[0]), min(current_box[1], adj_box[1]))
                

                # this might be wrong so beware
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
                '''
                print('\n')
                print(current_box)
                print(adj_box)
                print(current_point)
                print(detail_point)
                print(x_range)
                print(y_range)
                print('\n')
                '''
                detail_points[adj_box] = detail_point
                box_point[detail_point] = adj_box
                queue.append(adj_box)