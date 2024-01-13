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
    print(source_point)

    start = findBox(source_point, mesh)        #variable to hold source (first click) location
    dest = findBox(destination_point, mesh)    #variable to hold destination (second click) location

    path = []
    path_queue = Queue()
    boxes = {}

    path_queue, boxes = BFS(start, dest, mesh)



    return path, boxes


def findBox(point, mesh):
    pointX = point[0]
    pointY = point[1]

    for coords in mesh['boxes']:
        if coords[0] <= pointX and pointX < coords[1]:
            if coords[2] <= pointY and pointY < coords[3]:
                return coords
            


def BFS(source, destination, mesh):
    #Make a frontier Queue and add the source point in
    explored = Queue()
    explored.put(source)

    #Dictionary to hold nodes visited starting with the Source
    from_nodes = dict()
    from_nodes[source] = None

    #While the explored queue is not empty get the first element of the queue
    while not explored.empty():
        current_node = explored.get()

        if current_node == destination:
            break

        #for each node in the mesh grid's' current node' adjacency list
        #if the node is not in visited nodes from the source
        #put it in the explored Queue and the from nodes dictionary
        for node in mesh['adj'][current_node]:
            if node not in from_nodes:
                explored.put(node)
                from_nodes[node] = current_node

    
    if destination not in from_nodes.keys():
        print("There is no path!")
        return
    

    return explored, from_nodes



#def testing ():    
#    my_dict= {'boxes': [(0,1,1,0), (1,2,2,0), (2,4,3,1), (4,5,3,0)], 'adj':{(0,1,1,0):[(1,2,2,0)], (1,2,2,0):[(0,1,1,0), (2,4,3,1)] ,(2,4,3,1): [(1,2,2,0), (4,5,3,0)], (4,5,3,0): [(2,4,3,1)] }}
#    start = (0,1,1,0)
#    stop = (2,4,3,1)
#    BFS(start, stop, my_dict)

#if __name__ == '__main__':
#    testing()    