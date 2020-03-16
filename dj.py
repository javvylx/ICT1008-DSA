import json
import heapq as hq
import math


def importFiles(edgepath, nodepath):
    with open(nodepath) as f:  # data/walknodes.geojson
        nodes = json.load(f)

    with open(edgepath) as f:  # data/walkedges.geojson
        edges = json.load(f)

    # return as dicts/list
    return nodes, edges


def convertToCoord(path, nodes):
    route = []
    for id in path:
        route.append([nodes[id][0]['lat'], nodes[id][0]['lon']])

    return route
    # return a list of coordinates to plot


# A* algorithm (priority)f(s) = (cost)g(s) + h(s)(estimation of remaining cost - heuristic)
def heuristic(x1, y1, x2, y2):
    # euclidean distance heuristic but omit square root to improve performance.
    # non-admissible as distance could be more than the heuristic of original start to end
    return math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2)


def dijkstra(start, end, edgesdict, nodesdict):
    heap = [(0, start, [])]  # cost, current node, path
    hq.heapify(heap)  # heapify the list (Just to make sure)
    visited = []  # visited nodes
    dist = {start: 0}  # initialize all other nodes distance to infinity
    while True:
        total_dist, cur_vertex, cur_path = hq.heappop(heap)
        # check if current vertex has been visited
        if cur_vertex in visited or cur_vertex not in edgesdict:
            continue
        # else append into visited list
        visited.append(cur_vertex)
        new_path = cur_path + [cur_vertex]
        # return condition
        if cur_vertex == end:
            return cur_path
        for node, cur_dist in edgesdict[cur_vertex]:
            if node not in dist or total_dist + cur_dist < dist[node]:
                dist[node] = cur_dist + total_dist
                priority = cur_dist + total_dist + heuristic(nodesdict[node][0]['lat'], nodesdict[node][0]['lon'],
                                                             nodesdict[end][0]['lat'], nodesdict[node][0]['lon'])
                hq.heappush(heap, (priority, node, new_path))
