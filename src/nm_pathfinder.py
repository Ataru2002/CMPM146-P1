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
    # testcode
    start = 0
    start2 = 0
    detail_points = {}
    point_construction(detail_points)
    # Extract the graph out of the mesh
    graph = {}
    for looper in mesh.values():
        if start == 1:
            for key in looper.keys():
                graph[key] = looper[key]
        start += 1
    
    path = []
    boxes = {}

    return path, boxes.keys()


def point_construction(test):
    # construction
    test.append(5)