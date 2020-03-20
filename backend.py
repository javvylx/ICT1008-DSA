import math
import json
import heapq as hq
import networkx as nx
import osmnx as ox
import folium as fol
import geopandas as gpd
import json
import datetime

punggol = gpd.read_file('geojson_files/map.geojson')
polygon = punggol['geometry'].iloc[0]
lrtGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, infrastructure='way["railway"]')
walkGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="walk")
BuildingsNodesFootPrint = ox.footprints.footprints_from_polygon(polygon, footprint_type='building',
                                                                retain_invalid=False)
driveGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="drive")
plotBuildingNodes = ox.project_gdf(BuildingsNodesFootPrint)

walkgraph = ox.graph_from_point((1.402777, 103.906493), distance=1500, network_type='walk', truncate_by_edge=True, simplify=False)
drivegraph = ox.graph_from_point((1.402777, 103.906493), distance=1500, network_type='drive_service', truncate_by_edge=True,
                                 simplify=False)

# Convert a graph into node and/or edge GeoDataFrames
walkNode, walkEdge = ox.graph_to_gdfs(walkGraph)
driveNode, driveEdge = ox.graph_to_gdfs(driveGraph)
lrtNode, lrtEdge = ox.graph_to_gdfs(lrtGraph)

pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)

style_function = lambda feature: dict(
    color="#FFD97B",
    weight=1,
    opacity=0.8)

driveGraphL = fol.GeoJson(driveEdge, name="Public Roads", style_function=style_function)
driveGraphL.add_to(pgmap)

walkEdgeL = fol.GeoJson(walkEdge, name="Walking Path",  style_function=style_function)
walkEdgeL.add_to(pgmap)

lrtGraphL = fol.GeoJson(lrtEdge, name="LRT Track",  style_function=style_function)
lrtGraphL.add_to(pgmap)

fol.GeoJson(plotBuildingNodes, name='Buildings',  style_function=style_function).add_to(pgmap)
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


def calculateShortest(start_node, end_node, mode):
    if mode == "walking":
        path = dijkstra(start_node, end_node, walkedges, walknodes)
        # ox.plot_route_folium(pgmap, path, popup_attribute='length')
        # print(path)
        return convertToCoord(path, walknodes)
    elif mode == "driving":
        path = dijkstra(start_node, end_node, driveedges, drivenodes)
        # ox.plot_route_folium(pgmap, path, popup_attribute='length')
        # print(path)
        return convertToCoord(path, drivenodes)


start = (1.413006, 103.9073345)  # Sample start coordinates
end = (1.3956014, 103.9172982)  # Sample end coordinates
startmarker = fol.Marker(location=[1.413006, 103.913249], popup='<strong>Start Point</strong>')
startmarker.add_to(pgmap)
endmarker = fol.Marker(location=[1.3956014, 103.9172982], popup='<strong>End Point</strong>')
endmarker.add_to(pgmap)
start1 = (1.4133181, 103.9108787)
end1 = (1.3984, 103.9072)

startmarker1 = fol.Marker(location=[1.4133181, 103.9108787], popup='<strong>Start Point 1</strong>')
startmarker1.add_to(pgmap)
endmarker1 = fol.Marker(location=[1.3984, 103.9072], popup='<strong>End Point 1</strong>')
endmarker1.add_to(pgmap)
# start = (103.9073345, 1.4060506) # Sample start coordinates
# end = (103.9172982, 1.3956014) # Sample end coordinates
start_node = str(ox.get_nearest_node(walkGraph, start, method='haversine'))
print(start_node)

end_node = str(ox.get_nearest_node(walkGraph, end, method='haversine'))
print(end_node)

start_node1 = str(ox.get_nearest_node(driveGraph, start1, method='haversine'))
print(start_node1)

end_node1 = str(ox.get_nearest_node(driveGraph, end1, method='haversine'))
print(end_node1)
# start_node = '2274884613'
# end_node=  '3057389878'
# test = calculateShortest(start_node, end_node)
# print(test)
print("-----------------------------------------")
tt = calculateShortest(start_node, end_node, "walking")
print(tt)

print("-----------------------------------------")
ll = calculateShortest(start_node1, end_node1, "driving")
print(ll)
# WALK ROUTE
fol.PolyLine(tt, color="red", weight=2.5, opacity=1).add_to(pgmap)
# DRIVE ROUTE
fol.PolyLine(ll, color='#3388ff', weight=2.5, opacity=1).add_to(pgmap)

# origin_node = list(walkGraph.nodes)[2]
# destination_node = list(walkGraph.nodes)[-1]
# route = nx.shortest_path(pgmap, origin_node, destination_node)


currentDT = datetime.datetime.now()
print("Current time:")
print(currentDT.strftime("%H%M"))
# Get system time to coordinate which buses are available at the time
current_time = int(currentDT.strftime("%H%M"))
startPoint = "Aft Punggol Pt Stn"
endPoint = "Coral Edge Stn Exit B"
print("Start Point:\n", startPoint)
print("End Point:\n", endPoint)

# Load data from json
myStop = json.loads(open("geojson_files/myStop.json").read())
myServices = json.loads(open("geojson_files/myServices.json").read())
myRoutes = json.loads(open("geojson_files/myRoutes.json").read())

# Create tables
stopDescription = {stop["Description"]: stop for stop in myStop}
stopCodes = {stop["BusStopCode"]: stop for stop in myStop}

# Setting Route
routes = {}
for n in myRoutes:
    try:
        first_bus = int(n["WD_FirstBus"])
        last_bus = int(n["WD_LastBus"])
    except:
        continue

    # Checks if the bus is available at the user's system time
    if first_bus <= last_bus:
        if not (first_bus <= current_time <= last_bus):
            continue
    if first_bus > last_bus:
        if last_bus <= current_time <= first_bus:
            continue
    key = (n["ServiceNo"], n["Direction"])
    if key not in routes:
        routes[key] = []
    routes[key] += [n]

graph = {}
for service, path in routes.items():
    for route_index in range(len(path) - 1):
        key = path[route_index]["BusStopCode"]
        if key not in graph:
            graph[key] = set()
        graph[path[route_index]["BusStopCode"]].add(path[route_index + 1]["BusStopCode"])


# breadthFirst
def breadthFirst(graph, startPoint, endPoint):
    from queue import Queue
    seen = set()
    # Generate queue of paths
    queue = Queue()
    # Push path into queue
    queue.put([startPoint])
    while queue:
        # Retrieve first path from queue
        path = queue.get()
        # Retrieve last node from path
        node = path[-1]
        # Check the path
        if node == endPoint:
            return path
        if node in seen:
            continue
        seen.add(node)
        # Construct new path to push into the queue
        for adjacent in graph.get(node, []):
            new_path = list(path)
            new_path.append(adjacent)
            queue.put(new_path)


path = breadthFirst(graph, stopDescription[startPoint]["BusStopCode"], stopDescription[endPoint]["BusStopCode"])


def getService(i):
    for (service, direction), n in routes.items():
        for j in range(len(n)-1):
            if path[i] == n[j]["BusStopCode"] and path[i+1] == n[j+1]["BusStopCode"]:
                return service, stopCodes[path[i]]["Description"], stopCodes[path[i]]["Latitude"], \
                       stopCodes[path[i]]["Longitude"],  stopCodes[path[i + 1]]["Description"], \
                       stopCodes[path[i+1]]["Latitude"], stopCodes[path[i+1]]["Longitude"]


print("")
start_bus = None
end_bus = None
for i in range(len(path) - 1):
    print(getService(i))

    print(getService(i)[2])
    print(getService(i)[3])
    # mark bus stops on map
    # start point
    fol.Marker(location=[getService(i)[2], getService(i)[3]], popup=getService(i)[1], icon=fol.Icon(color='red', icon='info-sign')).add_to(pgmap)
    print(getService(i)[5])
    print(getService(i)[6])
    # end point
    fol.Marker(location=[getService(i)[5], getService(i)[6]], popup=getService(i)[4], icon=fol.Icon(color='red', icon='info-sign')).add_to(pgmap)

    busStart = (getService(i)[2], getService(i)[3])
    busEnd = (getService(i)[5], getService(i)[6])
    start_bus = str(ox.get_nearest_edge(drivegraph, busStart))
    end_bus = str(ox.get_nearest_edge(drivegraph, busEnd))
    print(start_bus, end_bus)
    # bus = calculateShortest(start_bus, end_bus, "driving")
    # print("=================================================================")
    # print(bus)

    # -------------------------------------------------------------------------------------------------
    # fol.PolyLine(([float("{0:.7f}".format(getService(i)[2])), float("{0:.7f}".format(getService(i)[3]))],
    #               [float("{0:.7f}".format(getService(i)[5])), float("{0:.7f}".format(getService(i)[6]))]),
    #              color='green', weight=2.5, opacity=1).add_to(pgmap)
    # -------------------------------------------------------------------------------------------------

    # if i == 0:
    #     start_bus = str(ox.get_nearest_node(drivegraph, busStart, method='haversine'))
    #     print("this is start")
    #     print(start_bus)
    # if i == int((len(path)-2)/4):
    #     end_bus = str(ox.get_nearest_node(drivegraph, busEnd, method='haversine'))
    #     print("this is end")
    #     print(end_bus)
    #
    # if start_bus and end_bus is not None:
    #     print("-----------------------------------------")
    #     print(start_bus, end_bus)
    #     bus = calculateShortest(start_bus, end_bus, "driving")
    #     fol.PolyLine(bus, color='green', weight=2.5, opacity=1).add_to(pgmap)

print(len(path), "stops")
pgmap.save('templates/gui_frontend.html')
