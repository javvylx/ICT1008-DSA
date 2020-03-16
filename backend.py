import math
import json
import heapq as hq
import networkx as nx
import osmnx as ox
import folium as fol
import geopandas as gpd

punggol = gpd.read_file('geojson_files/map.geojson')
polygon = punggol['geometry'].iloc[0]
lrtGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, infrastructure='way["railway"]')
walkGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="walk")
BuildingsNodesFootPrint = ox.footprints.footprints_from_polygon(polygon, footprint_type='building',
                                                                retain_invalid=False)
driveGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="drive")
plotBuildingNodes = ox.project_gdf(BuildingsNodesFootPrint)

# Convert a graph into node and/or edge GeoDataFrames
walkNode, walkEdge = ox.graph_to_gdfs(walkGraph)
driveNode, driveEdge = ox.graph_to_gdfs(driveGraph)
lrtNode, lrtEdge = ox.graph_to_gdfs(lrtGraph)

pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)

driveGraphL = fol.GeoJson(driveEdge, name="Public Roads")
driveGraphL.add_to(pgmap)

walkEdgeL = fol.GeoJson(walkEdge, name="Walking Path")
walkEdgeL.add_to(pgmap)

lrtGraphL = fol.GeoJson(lrtEdge, name="LRT Track")
lrtGraphL.add_to(pgmap)

fol.GeoJson(plotBuildingNodes, name='Buildings').add_to(pgmap)
fol.LayerControl(collapsed=True).add_to(pgmap)

logoIcon = fol.features.CustomIcon('images/siticon.png', icon_size=(40, 40))
SITtooltip = fol.Marker(location=[1.413006, 103.913249], popup='<strong>SIT New Punggol Campus</strong>', icon=logoIcon)
SITtooltip.add_to(pgmap)


def importFiles(edgepath, nodepath):
    with open(nodepath) as f:  # data/walknodes.geojson
        nodes = json.load(f)

    with open(edgepath) as f:  # data/walkedges.geojson
        edges = json.load(f)

    # return as dicts/list
    return nodes, edges


walknodes, walkedges = importFiles('geojson_files/walkedges.geojson', 'geojson_files/walknodes.geojson')
drivenodes, driveedges = importFiles('geojson_files/driveedges.geojson', 'geojson_files/drivenodes.geojson')


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


def calculateShortest(start_node, end_node):
    path = dijkstra(start_node, end_node, walkedges, walknodes)
    # ox.plot_route_folium(pgmap, path, popup_attribute='length')
    # print(path)
    return convertToCoord(path, walknodes)


start = (1.4060506, 103.9073345)  # Sample start coordinates
end = (1.3956014, 103.9172982)  # Sample end coordinates
# start = (103.9073345, 1.4060506) # Sample start coordinates
# end = (103.9172982, 1.3956014) # Sample end coordinates
start_node = str(ox.get_nearest_node(walkGraph, start, method='haversine'))
print(start_node)

end_node = str(ox.get_nearest_node(walkGraph, end, method='haversine'))
print(end_node)
# start_node = '2274884613'
# end_node=  '3057389878'
# test = calculateShortest(start_node, end_node)
# print(test)
print("-----------------------------------------")
tt = calculateShortest(start_node, end_node)
print(calculateShortest(start_node, end_node))

fol.PolyLine(tt, color="red", weight=2.5, opacity=1).add_to(pgmap)

# origin_node = list(walkGraph.nodes)[2]
# destination_node = list(walkGraph.nodes)[-1]
# route = nx.shortest_path(pgmap, origin_node, destination_node)

pgmap.save('templates/gui_frontend.html')
