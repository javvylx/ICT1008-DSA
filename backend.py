from math import sin, cos, sqrt, atan2, radians, acos
import math
import heapq as hq
import osmnx as ox
import folium as fol
import geopandas as gpd
import json
import datetime
from functools import partial

# Adding of punggol map polygon created from geojson.io
punggol = gpd.read_file('geojson_files/map.geojson')

# Init the polygon data from geometry section of the first row of the data frame
polygon = punggol['geometry'].iloc[0]
# Retrieved LRT information from OSMNX
lrtGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, infrastructure='way["railway"]')
# Retrieved walking information from OSMNX
walkGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="walk")
# Retrieved Building information from OSMNX
BuildingsNodesFootPrint = ox.footprints.footprints_from_polygon(polygon, footprint_type='building',
                                                                retain_invalid=False)
# Retrieved Driving information from OSMNX
driveGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="drive")

# Plot Building Nodes using OSMNX projection
plotBuildingNodes = ox.project_gdf(BuildingsNodesFootPrint)

# Retrieved the walking nodes locations, 1.5km around punggol area
walkgraph = ox.graph_from_point((1.402777, 103.906493), distance=1500, network_type='walk', truncate_by_edge=True,
                                simplify=False)
# Retrieved drive_service roads nodes and edges, 1.5km around punggol area
drivegraph = ox.graph_from_point((1.402777, 103.906493), distance=1500, network_type='drive_service',
                                 truncate_by_edge=True,
                                 simplify=False)

# Convert a graph into node and/or edge GeoDataFrames
walkNode, walkEdge = ox.graph_to_gdfs(walkGraph)
walk = list(walkGraph.nodes.values())
driveNode, driveEdge = ox.graph_to_gdfs(driveGraph)
lrtNode, lrtEdge = ox.graph_to_gdfs(lrtGraph)


# Display SIT marker on map
def sitMarker(pgmap):
    logoIcon = fol.features.CustomIcon('images/siticon.png', icon_size=(40, 40))
    SITtooltip = fol.Marker(location=[1.413006, 103.913249], popup='<strong>SIT New Punggol Campus</strong>',
                            icon=logoIcon)
    SITtooltip.add_to(pgmap)


# Init Base Map
basePgMap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                           truncate_by_edge=True)
sitMarker(basePgMap)

# Setting of style for all the nodes, edges, etc
style_function = lambda feature: dict(
    color="#FFD97B",
    weight=1,
    opacity=0.8)

# Creation of map overlays
driveGraphL = fol.GeoJson(driveEdge, name="Public Roads", style_function=style_function)
driveGraphL.add_to(basePgMap)

walkEdgeL = fol.GeoJson(walkEdge, name="Walking Path", style_function=style_function)
walkEdgeL.add_to(basePgMap)

lrtGraphL = fol.GeoJson(lrtEdge, name="LRT Track", style_function=style_function)
lrtGraphL.add_to(basePgMap)

fol.GeoJson(plotBuildingNodes, name='Buildings', style_function=style_function).add_to(basePgMap)
fol.LayerControl(collapsed=True).add_to(basePgMap)

basePgMap.save('templates/gui_frontend.html')


def importFiles(edgepath, nodepath):
    with open(nodepath) as f:  # geojson_files/walknodes.geojson | geojson_files/drivenodes.geojson
        nodes = json.load(f)

    with open(edgepath) as f:  # geojson_files/walkedges.geojson | geojson_files/driveedges.geojson
        edges = json.load(f)

    # return as dicts/list
    return nodes, edges


walknodes, walkedges = importFiles('geojson_files/walkedges.geojson', 'geojson_files/walknodes.geojson')
drivenodes, driveedges = importFiles('geojson_files/driveedges.geojson', 'geojson_files/drivenodes.geojson')

# Convert long n lat to to coordinates
def convertToCoord(path, nodes):
    route = []
    for id in path:
        route.append([nodes[id][0]['lat'], nodes[id][0]['lon']])
    return route
    # return a list of coordinates to plot

# convert OSMID to Long and Lat
def convertOSMIDtoLL(userInput):
    for k in walk:
        if k.get("osmid") == userInput:
            return str(k.get("x")), str(k.get("y"))

# A Star Heuristic
def heuristic(x1, y1, x2, y2):
    return math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2)

# Dijkstra Heuristic
def dijkstra(start, end, edgesdict, nodesdict):
    heap = [(0, start, [])]  # cost, current node, path
    hq.heapify(heap)  # creates a heap
    visited = []
    dist = {start: 0}  # initialize all distance to infinity
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
            # pushes to path


# array for west lrt station edges
westlrtstat = [[
    1.4055940063199879,
    103.90233993530273
],
    [
        1.409026198275084,
        103.90448570251465
    ], [
        1.415204131042014,
        103.90774726867676
    ], [
        1.409026198275084,
        103.90448570251465
    ],
    [
        1.415204131042014,
        103.90774726867676
    ],
    [
        1.4166628072131071,
        103.90736103057861
    ],
    [
        1.417091829441639,
        103.9063310623169
    ],
    [
        1.4175852049061857,
        103.90470027923584
    ],
    [
        1.4164911982994481,
        103.90240430831909
    ],
    [
        1.4147965595948906,
        103.90133142471312
    ],
    [
        1.4118255760945477,
        103.90031218528748
    ],
    [
        1.408543546586684,
        103.8985526561737
    ],
    [
        1.405358042937595,
        103.89726519584656
    ],
    [
        1.4036848473606192,
        103.89682531356812
    ],
    [
        1.402290516798469,
        103.90023708343506
    ],
    [
        1.402934054084264,
        103.90111684799194
    ],
    [
        1.4055940063199879,
        103.90233993530273
    ]
]
# array for east lrt station edges
eastlrtstat = [[
    1.4055940063199879,
    103.90233993530273
],
    [
        1.4025264804904696,
        103.90090227127075
    ],
    [
        1.4017971380928607,
        103.90124559402466
    ],
    [
        1.3995662069962271,
        103.90588045120239
    ],
    [
        1.3970135043880965,
        103.90892744064331
    ],
    [
        1.393817259397357,
        103.91270399093628
    ],
    [
        1.3927446929915568,
        103.91424894332886
    ],
    [
        1.3929806576427364,
        103.91491413116455
    ],
    [
        1.3946538608547738,
        103.91611576080321
    ],
    [
        1.3974425302305653,
        103.91811132431029
    ],
    [
        1.3983434842446176,
        103.91811132431029
    ],
    [
        1.3996734633475383,
        103.91637325286865
    ],
    [
        1.402397773025064,
        103.91266107559204
    ],
    [
        1.405358042937595,
        103.90849828720093
    ],
    [
        1.408189601951655,
        103.90433549880981
    ],
    [
        1.407739126883896,
        103.90356302261353
    ],
    [
        1.4055940063199879,
        103.90233993530273
    ]
]
# list for all west lrt stations for duration from their lrt station to its next lrt station to the right
westduration = [2, 2, 1, 1, 1, 1, 3]
# list for all east lrt stations for duration from their lrt station to its next lrt station to the right
eastduration = [3, 1, 1, 1, 1, 1, 1, 2]
# array for all west lrt stops
westlrtnodes = [[1.4055940063199879, 103.90233993530273], [1.409026198275084, 103.90448570251465],
                [1.417091829441639, 103.9063310623169], [1.4164911982994481, 103.90240430831909],
                [1.4118255760945477, 103.90031218528748], [1.408543546586684, 103.8985526561737],
                [1.405358042937595, 103.89726519584656]]
# array for all east lrt stops
eastlrtnodes = [[1.4055940063199879, 103.90233993530273], [1.3995662069962271, 103.90588045120239],
                [1.3970135043880965, 103.90892744064331], [1.393817259397357, 103.91270399093628],
                [1.3946538608547738, 103.91611576080321], [1.3996734633475383, 103.91637325286865],
                [1.402397773025064, 103.91266107559204], [1.405358042937595, 103.90849828720093]]
# list of the name of west lrt stations
wlrt = ["Punggol", "Sam Kee", "Punggol Point", "Samudera", "Nibong", "Sumang", "Soo Teck"]
# list of the name of east lrt stations
elrt = ["Punggol", "Cove", "Meridian", "Coral Edge", "Riveria", "Kadaloor", "Oasis", "Damai"]
# Markers for all the lrt stations
ptc = fol.Marker(location=[1.4055940063199879, 103.90233993530273], popup='<strong>Punggol</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pw1 = fol.Marker(location=[1.409026198275084, 103.90448570251465], popup='<strong>Sam Kee</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pw3 = fol.Marker(location=[1.417091829441639, 103.9063310623169], popup='<strong>Punggol Point</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pw4 = fol.Marker(location=[1.4164911982994481, 103.90240430831909], popup='<strong>Samudera</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pw5 = fol.Marker(location=[1.4118255760945477, 103.90031218528748], popup='<strong>Nibong</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pw6 = fol.Marker(location=[1.408543546586684, 103.8985526561737], popup='<strong>Sumang</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pw7 = fol.Marker(location=[1.405358042937595, 103.89726519584656], popup='<strong>Soo Teck</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe1 = fol.Marker(location=[1.3995662069962271, 103.90588045120239], popup='<strong>Cove</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe2 = fol.Marker(location=[1.3970135043880965, 103.90892744064331], popup='<strong>Meridian</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe3 = fol.Marker(location=[1.393817259397357, 103.91270399093628], popup='<strong>Coral Edge</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe4 = fol.Marker(location=[1.3946538608547738, 103.91611576080321], popup='<strong>Riveria</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe5 = fol.Marker(location=[1.3996734633475383, 103.91637325286865], popup='<strong>Kadaloor</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe6 = fol.Marker(location=[1.402397773025064, 103.91266107559204], popup='<strong>Oasis</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
pe7 = fol.Marker(location=[1.405358042937595, 103.90849828720093], popup='<strong>Damai</strong>',
                 icon=fol.Icon(color='black', icon='train', prefix='fa'))
# Used for pinning  to the map purposes
pintag = []
# list of pins of all the west lrt stations
westpin = [ptc, pw1, pw3, pw4, pw5, pw6, pw7]
# list of pins of all the east lrt stations
eastpin = [ptc, pe1, pe2, pe3, pe4, pe5, pe6, pe7]
# used to calculate total duration to go from one lrt to another
lrttime = 0
lrtstops = []


def lrtrouting(start, end, lrtnodes, duration, lrtstat, pin):
    forward = 0  # used to count time when the lrt goes the right side till it reaches its destination
    reverse = 0  # used to count time when the lrt goes the left side till it reaches its destination
    append = 0  # works like routing permission key, if 0 means routing not in process if 1 means in process
    test = []
    sp = 0
    ep = 0
    node = []
    for i in range(len(lrtnodes)):  # to save the index number of the start point and end point
        if start == lrtnodes[i]:
            sp = i

        elif end == lrtnodes[i]:
            ep = i
        else:
            continue
    if ep < sp:  # if end point less than start point swap for better routing
        temp = sp
        temp2 = start
        sp = ep
        start = end
        ep = temp
        end = temp2
    for i in range(len(lrtnodes)):
        if i >= sp and i < ep:  # when lrt goes forward/right
            forward = forward + duration[i]
        else:  # when lrt goes reverse/left
            reverse = reverse + duration[i]
    if forward <= reverse:
        lrttime = forward
        for i in range(len(lrtstat)):
            if start == lrtstat[i]:  # the edge for the start point is found, so now the routing can begin
                append = 1
            if append == 1:
                node.append(lrtstat[i])
            if end == lrtstat[i]:  # when edge for the endpoint found, the routing stops
                append = 0
                break
    else:
        lrttime = reverse
        for i in range(len(
                lrtstat)):  # the edge for the end point is found, so now the routing can begin, remember this is working backwards
            if end == lrtstat[i]:
                append = 1
            if append == 1:
                node.append(lrtstat[i])
        for i in range(len(lrtstat)):
            if append == 1:
                node.append(lrtstat[i])
            if start == lrtstat[i]:  # when edge for the startpoint found, the routing stops
                append = 0
                break

    for i in range(len(node)):
        for j in range(len(lrtnodes)):
            if node[i] == lrtnodes[j]:  # to put the marker on the lrts that will pass by from start to end
                pintag.append(pin[j])

    return node


def lrtroute(start, end):  # before routing the lrt, find out which station it belongs to
    global path1, path2
    west = 0
    east = 0
    for i in range(len(westlrtnodes)):  # compare if start/end found in west section
        if start == westlrtnodes[i]:  # if start is from west lrt then west increment by 1
            west = west + 1
            stw = 'w'  # to show where start is from
        if end == westlrtnodes[i]:  # if end is from west lrt then west increment by 1
            west = west + 1
    for i in range(len(eastlrtnodes)):  # compare if start/end found in east section
        if start == eastlrtnodes[i]:  # if start is from east lrt then east increment by 1
            east = east + 1
            stw = 'e'  # to show where end is from
        if end == eastlrtnodes[i]:  # if end is from east lrt then west increment by 1
            east = east + 1
    if west == 2:  # if both start and end are on west lrt, we do routing just on west side once
        return lrtrouting(start, end, westlrtnodes, westduration, westlrtstat, westpin)
    if east == 2:  # if both start and end are on east lrt, we do routing just on east side once
        return lrtrouting(start, end, eastlrtnodes, eastduration, eastlrtstat, eastpin)
    if west == 1 and east == 1:  # if start and end are from different side we do routing for both
        combopath = []
        if stw == 'w':  # if start is from west, then we do routing from west stn to punggol and then routing from punggol to east stn
            path1 = lrtrouting(start, westlrtnodes[0], westlrtnodes, westduration, westlrtstat, westpin)
            path2 = lrtrouting(eastlrtnodes[0], end, eastlrtnodes, eastduration, eastlrtstat, eastpin)
        if stw == 'e':  # if start is from east, then we do routing from east stn to punggol and then routing from punggol to west stn
            path1 = lrtrouting(start, eastlrtnodes[0], eastlrtnodes, eastduration, eastlrtstat, eastpin)
            path2 = lrtrouting(westlrtnodes[0], end, westlrtnodes, westduration, westlrtstat, westpin)
        if path1[0] == [1.4055940063199879, 103.90233993530273]:  # if path1 begins from punggol
            for i in range(0, len(path1) // 2):  # swap to avoid conflict in routing
                temp = path1[i]
                path1[i] = path1[len(path1) - 1 - i]
                path1[len(path1) - 1 - i] = temp
        for i in range(len(path1)):  # add the arranged path to the combined path later used in mapping
            combopath.append(path1[i])
        if path2[-1] == [1.4055940063199879, 103.90233993530273]:  # if path2 ends with punggol
            for i in range(0, len(path2) // 2):  # swap to avoid conflict in routing
                temp = path2[i]
                path2[i] = path2[len(path2) - 1 - i]
                path2[len(path2) - 1 - i] = temp
        for i in range(len(path2)):  # add the arranged path to the combined path later used in mapping
            combopath.append(path2[i])
        # print(combopath)
        return combopath


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
    elif mode == "lrt":
        path = lrtroute(start_node,
                        end_node)  # call the lrtroute function to find the stations and then that will call the lrt routing function
        return path


def calculateDist(lat1, long1, lat2, long2):  # calculates the distance between coordinates
    radius = 6373.0
    lat1_ = radians(lat1)
    long1_ = radians(long1)
    lat2_ = radians(lat2)
    long2_ = radians(long2)
    dlon = long2_ - long1_
    dlat = lat2_ - lat1_
    a = sin(dlat / 2) ** 2 + cos(lat1_) * cos(lat2_) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = radius * c
    return distance


def getNearestEdgeNode(osm_id, a):  # gets nearest edge from the node from the osmid in question
    y = a.nodes.get(osm_id).get('y')
    x = a.nodes.get(osm_id).get('x')
    nearest_edge = ox.get_nearest_edge(a, (y, x))
    temp1 = nearest_edge[0].coords[0]
    temp2 = nearest_edge[0].coords[1]
    temp1x = temp1[0]
    temp1y = temp1[1]
    temp1dist = calculateDist(temp1y, temp1x, y, x)
    temp2x = temp2[0]
    temp2y = temp2[1]
    temp2dist = calculateDist(temp2y, temp2x, y, x)
    if temp1dist < temp2dist:
        return nearest_edge[1]
    else:
        return nearest_edge[2]


# Get system time to coordinate which buses are available at the time
# ---------------------------------------------------------------------------------
currentDT = datetime.datetime.now()
current_time = int(currentDT.strftime("%H%M"))  # stores current time to reference with the bus timings
# -------------------------------------------------------------------------------
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


# Main Bus routing Function
# Walking portion uses the A star + dijkstra algorithm. Bus uses breadth first search as the routing for the bus is
# already set from the LTA database, therefore allows us to do a search to find all possible routes, singling out the
# single route that can be used for the routing. From the coordinate chosen, the program finds the nearest bus stop
# coordinate by calculating the shortest distance between each possibility, then using the walking routes, plots a path
# to the bus stop. The program then runs the bus routing to the bus stop closest to the end coordinate and the walk
# routing runs again to plot to the end coordinate.

def busRouting(startPoint, endPoint, pgmap):
    # get service returns the description from the path into a tuple
    def getService(i):
        for (service, direction), n in routes.items():
            for j in range(len(n) - 1):
                if path[i] == n[j]["BusStopCode"] and path[i + 1] == n[j + 1]["BusStopCode"]:
                    return service, stopCodes[path[i]]["Description"], stopCodes[path[i]]["Latitude"], \
                           stopCodes[path[i]]["Longitude"], stopCodes[path[i + 1]]["Description"], \
                           stopCodes[path[i + 1]]["Latitude"], stopCodes[path[i + 1]]["Longitude"]

    # running breadth first to get the path
    path = breadthFirst(graph, stopDescription[startPoint]["BusStopCode"], stopDescription[endPoint]["BusStopCode"])
    # print("")
    start_bus = None
    end_bus = None
    last_node = None
    busSummary = []
    dist = 0
    # running get service in order to retrieve the description such as bus number, description and coordinates
    for i in range(len(path) - 1):
        # print(getService(i))
        #
        # print(getService(i)[2])
        # print(getService(i)[3])
        # mark bus stops on map
        # start point
        fol.Marker(location=[getService(i)[2], getService(i)[3]], tooltip=getService(i)[1],
                   popup="Bus No. " + getService(i)[0],
                   icon=fol.Icon(color='green', icon='bus', prefix='fa')).add_to(pgmap)
        # print(getService(i)[5])
        # print(getService(i)[6])
        # end point
        fol.Marker(location=[getService(i)[5], getService(i)[6]], tooltip=getService(i)[4],
                   popup="Bus No. " + getService(i)[0],
                   icon=fol.Icon(color='green', icon='bus', prefix='fa')).add_to(pgmap)

        busStart = (getService(i)[2], getService(i)[3])
        busEnd = (getService(i)[5], getService(i)[6])
        start_bus = str(ox.get_nearest_node(drivegraph, busStart))
        end_bus = str(ox.get_nearest_node(drivegraph, busEnd))
        # print(start_bus, end_bus)
        if start_bus == end_bus:
            fol.PolyLine(([float("{0:.7f}".format(getService(i)[2])), float("{0:.7f}".format(getService(i)[3]))],
                          [float("{0:.7f}".format(getService(i)[5])), float("{0:.7f}".format(getService(i)[6]))]),
                         color='green', weight=2.5, opacity=1).add_to(pgmap)
            pgmap.save('templates/gui_frontend.html')
            continue
        else:
            start_bus_x = drivenodes.get(start_bus)[0].get('lat')
            start_bus_y = drivenodes.get(start_bus)[0].get('lon')
            end_bus_x = drivenodes.get(end_bus)[0].get('lat')
            end_bus_y = drivenodes.get(end_bus)[0].get('lon')
            bus = calculateShortest(start_bus, end_bus, "driving")
            # print(bus)
            # plots the routes from bus stop to bus stops using the dijkstra alg
            # calculates distance from each path
            for m in range(len(bus) - 1):
                dist = dist + calculateDist(float(bus[m][0]), float(bus[m][1]), float(bus[m + 1][0]),
                                            float(bus[m + 1][1]))

            if last_node is not None:
                fol.PolyLine(([last_node[0], last_node[1]],
                              [bus[0][0], bus[0][1]]),
                             color='green', weight=2.5, opacity=1).add_to(pgmap)
            # connects the polyline if the routes do not connect to each other (GUI)
            fol.PolyLine(([start_bus_x, start_bus_y],
                          [getService(i)[2], getService(i)[3]]),
                         color='green', weight=2.5, opacity=1).add_to(pgmap)
            fol.PolyLine(([getService(i)[5], getService(i)[6]], [end_bus_x, end_bus_y]),
                         color='green', weight=2.5, opacity=1).add_to(pgmap)
            # pgmap.save('templates/gui_frontend.html')

            last_node = bus[-1]
            # start_bus_edge = getNearestEdgeNode(start_bus, drivegraph)
            # end_bus_edge = getNearestEdgeNode(end_bus, drivegraph)

            # print(bus)
            fol.PolyLine(bus, color='green', weight=2.5, opacity=1).add_to(pgmap)
            busSummary.append(getService(i))
            # plots markers for start and end conditionals
            if i == 0:
                fol.Marker(location=[getService(i)[2], getService(i)[3]], tooltip=getService(i)[1],
                           popup="Bus No. " + getService(i)[0],
                           icon=fol.Icon(color='blue', icon='bus', prefix='fa')).add_to(pgmap)
                # pgmap.save('templates/gui_frontend.html')
            if i == len(path) - 2:
                fol.Marker(location=[getService(i)[5], getService(i)[6]], tooltip=getService(i)[4],
                           popup="Bus No. " + getService(i)[0],
                           icon=fol.Icon(color='red', icon='bus', prefix='fa')).add_to(pgmap)
    pgmap.save('templates/gui_frontend.html')
    # bus travels approximately 35km/h, with the waiting time for every stop
    time = dist / 35 * 60
    cleanDist = str(round(dist, 2))
    cleanTime = str(round(time, 2))
    # print(cleanDist)
    # print(cleanTime)
    busArray = [cleanDist, cleanTime]  # places distance and time in an array to be returned
    # print(busSummary)
    # print(len(path), "stops")
    return busSummary, len(path), busArray


def find_nearest(points, coor):
    dist = lambda s, key: (s[0] - points[key][0]) ** 2 + (s[1] - points[key][1]) ** 2
    return min(points, key=partial(dist, coor))


# find nearest is used to find the nearest station to the coordinate in place in order to do
# either walk + bus or walk + lrt


keys = []
values = []
for i in range(len(myStop)):
    keys.append(myStop[i].get("Description"))
    t = ()
    t = (myStop[i].get('Latitude'), myStop[i].get('Longitude'))
    values.append(t)

graph_dict = dict(zip(keys, values))


# creates a dictionary of all buses in order to use the find_nearest function


# walkPlusBus routes the user from the coordinate of his choice to the nearest bus station using the walking
# algorithm. then running the bus algorithm to the bus stop nearest to the end point, routing the walking path to the
# end point
def walkPlusBus(src1, des1):
    pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)
    sitMarker(pgmap)
    startbus1coord = (float(ox.geocode(src1)[0]), float(ox.geocode(src1)[1]))
    endbus1coord = (float(ox.geocode(des1)[0]), float(ox.geocode(des1)[1]))
    # print(startbus1coord)
    # print(endbus1coord)
    startbus1 = find_nearest(graph_dict, startbus1coord)
    # print(startbus1)
    endbus1 = find_nearest(graph_dict, endbus1coord)
    # print(endbus1)

    start_bus_coord = graph_dict.get(startbus1)
    end_bus_coord = graph_dict.get(endbus1)

    start_node1 = str(ox.get_nearest_node(walkgraph, startbus1coord, method='haversine'))
    end_node1 = str(ox.get_nearest_node(walkgraph, endbus1coord, method='haversine'))

    start_bus_node = str(ox.get_nearest_node(walkgraph, start_bus_coord, method='haversine'))
    end_bus_node = str(ox.get_nearest_node(walkgraph, end_bus_coord, method='haversine'))

    start_to_bus = calculateShortest(start_node1, start_bus_node, "walking")
    bus_to_end = calculateShortest(end_bus_node, end_node1, "walking")

    walktobusStatement = "Walk from Start Point: " + src1 + " to " + str(startbus1) + " Bus Stop"
    bustoendStatement = "Walk from " + str(endbus1) + " Bus Stop to End Point: " + des1

    data = busRouting(startbus1, endbus1, pgmap)

    fol.Marker(location=[startbus1coord[0], startbus1coord[1]], popup=src1, tooltip="Start Point",
               icon=fol.Icon(color='blue')).add_to(pgmap)
    fol.Marker(location=[endbus1coord[0], endbus1coord[1]], popup=des1, tooltip='End Point',
               icon=fol.Icon(color='red')).add_to(pgmap)
    fol.PolyLine(
        ([startbus1coord[0], startbus1coord[1]],
         [walknodes.get(start_node1)[0].get('lat'), walknodes.get(start_node1)[0].get('lon')]),
        color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)
    fol.PolyLine(([walknodes.get(end_node1)[0].get('lat'), walknodes.get(end_node1)[0].get('lon')],
                  [endbus1coord[0], endbus1coord[1]]),
                 color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)
    fol.PolyLine(start_to_bus, color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)
    fol.PolyLine(bus_to_end, color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)
    pgmap.save('templates/gui_frontend.html')
    # calculates walking distance from start till the start bus stop
    dist = 0
    for m in range(len(start_to_bus) - 1):
        dist = dist + calculateDist(float(start_to_bus[m][0]), float(start_to_bus[m][1]), float(start_to_bus[m + 1][0]),
                                    float(start_to_bus[m + 1][1]))
    time = dist / 5 * 60
    cleanDist = str(round(dist, 2))
    cleanTime = str(round(time, 2))
    walktobusDist = [cleanDist, cleanTime]
    # calculates walking distance from end bus stop till the end
    dist1 = 0
    for m in range(len(bus_to_end) - 1):
        dist1 = dist1 + calculateDist(float(bus_to_end[m][0]), float(bus_to_end[m][1]), float(bus_to_end[m + 1][0]),
                                      float(bus_to_end[m + 1][1]))
    time1 = dist1 / 5 * 60
    cleanDist1 = str(round(dist1, 2))
    cleanTime1 = str(round(time1, 2))
    bustoendDist = [cleanDist1, cleanTime1]
    pgmap.save('templates/gui_frontend.html')
    return data, walktobusStatement, bustoendStatement, walktobusDist, bustoendDist

# Main function for walking algorithm
# Uses the A star + dijkstra algorithm, utilising 2 dictionaries consisting of edges and nodes for either driving or
# walking, checking the whether the nodes are visited or not, using heuristic, adding the distance to create the priority
# of the node, pushing to the heap and popping to append to the path with the corresponding edges.

def walking(src, des):
    pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)
    sitMarker(pgmap)
    startpoint = ox.geocode(src)
    endpoint = ox.geocode(des)
    # m = folium.map(location=punggol, distance=distance)
    startmarker = fol.Marker(location=startpoint, popup='<strong>' + src + '</strong>',
                             tooltip="Start", icon=fol.Icon(color='blue', icon='male', prefix='fa'))
    startmarker.add_to(pgmap)
    endmarker = fol.Marker(location=endpoint, popup='<strong>' + des + '</strong>',
                           tooltip="End", icon=fol.Icon(color='red', icon='male', prefix='fa'))
    endmarker.add_to(pgmap)
    start_node = str(ox.get_nearest_node(walkGraph, startpoint, method='haversine'))
    # print(start_node)
    end_node = str(ox.get_nearest_node(walkGraph, endpoint, method='haversine'))
    # print(end_node)
    # print("-----------------------------------------")
    tt = calculateShortest(start_node, end_node, "walking")
    # print(tt)
    dist = 0
    for m in range(len(tt) - 1):
        dist = dist + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]), float(tt[m + 1][1]))
    time = dist / 5 * 60
    cleanDist = str(round(dist, 2))
    cleanTime = str(round(time, 2))
    # print(cleanDist)
    # print(cleanTime)
    walkArray = [cleanDist, cleanTime]
    # WALK ROUTE
    fol.PolyLine(tt, color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)

    pgmap.save('templates/gui_frontend.html')

    return walkArray

# For the LRT section, a walk path is implemented using dijkstra to find the nearest lrt stations from its start and end location.
# Then for the LRT stops found, a linear search is done for both the stations to determine whether they belong to the same line.
# LRT stations on a line usually have 2 neighbours(the next stops).
# If they belong to the same LRT line, for better accuracy two possible routes are calculated from its two different
# neighbours based on its travel time and from there Prim's algorithm is carried out to prevent creating a loop.
# However if the stations belong to a different line, the process for used for the same lrt scenario is implemented twice,
# one for start and one for end just that Punggol will be the end and start point of the respective process, as Punggol
# is the station linking the two lines, hence Punggol is the only station with 4 neighbours.But if the focus is just on
# the same LRT line only, Punggol has two neighbbours too, since there isn't a need for the
# other two neighbours in this process.
# After the walk paths and lrt path are figured out, they are joined together to display the final routing on the map.
def walkforlrt(src, des):
    alrt = []
    stops = []
    countway = 0  # to count how many lrt stations to start/stop at
    pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)
    sitMarker(pgmap)
    # to do routing from start destination to the nearest lrt station
    # start destination as per normal
    startpoint = ox.geocode(src)
    startmarker = fol.Marker(location=startpoint, popup='<strong>' + src + '</strong>',
                             tooltip="Start", icon=fol.Icon(color='blue', icon='male', prefix='fa'))
    startmarker.add_to(pgmap)
    start_node = str(ox.get_nearest_node(walkGraph, startpoint, method='haversine'))
    # print(start_node)
    # assume nearest lrt station is the first lrt station(punggol)
    endpoint1 = eastlrtnodes[0]
    end_node1 = str(ox.get_nearest_node(walkGraph, endpoint1, method='haversine'))
    # print(end_node1)
    # print("-----------------------------------------")
    # find the distance from the start to lrt station(first lrt station in this case)
    tt = calculateShortest(start_node, end_node1, "walking")
    # print(tt)
    dist1 = 0
    lrtin1 = 0
    lrtin2 = 0
    # to calculate its distance and time
    for m in range(len(tt) - 1):
        dist1 = dist1 + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]), float(tt[m + 1][1]))
    time1 = dist1 / 5 * 60
    cleanDist1 = str(round(dist1, 2))
    cleanTime1 = str(round(time1, 2))
    # print(cleanDist1)
    # print(cleanTime1)
    # to create a list with all the lrt stations from east and west
    for a in range(1, len(eastlrtnodes)):
        alrt.append(eastlrtnodes[a])
        stops.append(elrt[a])
    for a in range(1, len(westlrtnodes)):
        alrt.append(westlrtnodes[a])
        stops.append(wlrt[a])
    for i in range(len(alrt)):
        try:
            # to check distance and time for all the lrt station from the start point and choose the nearest one
            endpoint2 = alrt[i]
            end_node2 = str(ox.get_nearest_node(walkGraph, endpoint2, method='haversine'))
            # print(end_node2)
            # print("-----------------------------------------")
            tt = calculateShortest(start_node, end_node2, "walking")
            # print(tt)
            dist2 = 0
            for m in range(len(tt) - 1):
                dist2 = dist2 + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]),
                                              float(tt[m + 1][1]))
            time2 = dist2 / 5 * 60
            cleanDist2 = str(round(dist2, 2))
            cleanTime2 = str(round(time2, 2))
            # print(cleanDist2)
            # print(cleanTime2)
            if (
                    cleanDist1 <= cleanDist2):  # if an lrt station with lesser distance found then that becomes the least one
                endpoint = endpoint1
            else:
                cleanDist1 = cleanDist2
                endpoint1 = endpoint2
                endpoint = endpoint2
                lrtin1 = i
        except KeyError:  # if any of them is out of range, ignore it
            continue

    # m = folium.map(location=punggol, distance=distance)
    way1 = stops[lrtin1]  # to setup the name of the nearest stn from start
    end_node = str(ox.get_nearest_node(walkGraph, endpoint, method='haversine'))
    # print(end_node)
    # print("-----------------------------------------")
    # do a final routing of start to its nearest lrt
    tt = calculateShortest(start_node, end_node, "walking")
    # print(tt)
    # distance and time from start to nearest lrt finalised
    dista = 0
    for m in range(len(tt) - 1):
        dista = dista + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]), float(tt[m + 1][1]))
    timea = dista / 5 * 60
    cleanDist = str(round(dista, 2))
    cleanTime = str(round(timea, 2))
    # print(cleanDist)
    # print(cleanTime)
    walkArray = [cleanDist, cleanTime]
    # WALK ROUTE for start to nearest lrt add to map
    fol.PolyLine(tt, color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)
    # now  do routing from end destination to the nearest lrt station
    temp = src
    src = des
    des = temp
    # swapping for src and des so that des acts like an src here(only for routing purposes)
    # does exactly what the src and start point did
    startpoint = ox.geocode(src)
    startmarker = fol.Marker(location=startpoint, popup='<strong>' + src + '</strong>',
                             tooltip="End", icon=fol.Icon(color='red', icon='male', prefix='fa'))
    startmarker.add_to(pgmap)

    end = str(ox.get_nearest_node(walkGraph, startpoint, method='haversine'))
    tt = calculateShortest(start_node, end, "walking")
    distc = 0
    for m in range(len(tt) - 1):
        distc = distc + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]), float(tt[m + 1][1]))

    start_node = str(ox.get_nearest_node(walkGraph, startpoint, method='haversine'))
    # print(start_node)

    endpoint1 = eastlrtnodes[0]
    end_node1 = str(ox.get_nearest_node(walkGraph, endpoint1, method='haversine'))
    # print(end_node1)
    # print("-----------------------------------------")
    tt = calculateShortest(start_node, end_node1, "walking")
    # print(tt)
    dist1 = 0
    for m in range(len(tt) - 1):
        dist1 = dist1 + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]), float(tt[m + 1][1]))
    time1 = dist1 / 5 * 60
    cleanDist1 = str(round(dist1, 2))
    cleanTime1 = str(round(time1, 2))
    # print(cleanDist1)
    # print(cleanTime1)
    for a in range(len(eastlrtnodes)):
        alrt.append(eastlrtnodes[a])
    for a in range(len(westlrtnodes)):
        alrt.append(westlrtnodes[a])
    for i in range(len(alrt)):
        try:
            endpoint2 = alrt[i]
            end_node2 = str(ox.get_nearest_node(walkGraph, endpoint2, method='haversine'))
            # print(end_node2)
            # print("-----------------------------------------")
            tt = calculateShortest(start_node, end_node2, "walking")
            # print(tt)
            dist2 = 0
            for m in range(len(tt) - 1):
                dist2 = dist2 + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]),
                                              float(tt[m + 1][1]))
            time2 = dist2 / 5 * 60
            cleanDist2 = str(round(dist2, 2))
            cleanTime2 = str(round(time2, 2))
            # print(cleanDist2)
            # print(cleanTime2)
            if cleanDist1 <= cleanDist2:
                endpoint = endpoint1
            else:
                cleanDist1 = cleanDist2
                endpoint1 = endpoint2
                endpoint = endpoint2
                lrtin2 = i
        except KeyError:
            continue
    # m = folium.map(location=punggol, distance=distance)
    # if the lrts are from different side, then there will be 3 stn involved means change from start to punggol, punggol to end
    if (lrtin1 != 0 or 8 and lrtin2 != 0 or 8):
        if (lrtin1 < 8 and lrtin2 > 8) or (lrtin1 > 8 and lrtin2 < 8):
            way2 = "Punggol"
            way3 = stops[lrtin2]
            countway = 3
        else:
            way2 = stops[lrtin2]
            countway = 2
    else:
        way2 = stops[lrtin2]
        countway = 2

    end_node = str(ox.get_nearest_node(walkGraph, endpoint, method='haversine'))
    # print(end_node)
    # print("-----------------------------------------")
    # now finalize the timing for the dest and its nearest stop
    tt = calculateShortest(start_node, end_node, "walking")
    # print(tt)
    distb = 0
    for m in range(len(tt) - 1):
        distb = distb + calculateDist(float(tt[m][0]), float(tt[m][1]), float(tt[m + 1][0]), float(tt[m + 1][1]))
    timeb = distb / 5 * 60
    cleanDist = str(round(distb, 2))
    cleanTime = str(round(timeb, 2))
    # print(cleanDist)
    # print(cleanTime)

    # WALK ROUTE from nearest dest stop to dest
    fol.PolyLine(tt, color="#3388ff", weight=2.5, opacity=1).add_to(pgmap)

    # final routing this time for lrt from nearest start stn to nearest end stn
    tt = calculateShortest(alrt[lrtin1], alrt[lrtin2], "lrt")
    pinning = []
    for x in pintag:  # check if exists in unique_list or not
        if x not in pinning:
            pinning.append(x)
    for i in range(len(pinning)):
        pinning[i].add_to(pgmap)

    # add the lrt routing to the map
    fol.PolyLine(tt, color="black", weight=2.5, opacity=1).add_to(pgmap)

    # finalise timing for lrt
    timec = lrttime
    if countway == 2:
        # add all the three timings increment by 2(waiting time for lrt to come from start stn)
        ftime = timea + timeb + timec + 2
    elif countway == 3:
        # add all the three timings increment by 5(waiting time for lrt to come from start stn, and also from punggpl stn when changing)
        ftime = timea + timeb + timec + 5
    # finalise distance and time
    fdist = distc
    cleanDist = str(round(fdist, 2))
    cleanTime = str(round(ftime, 2))
    if countway == 2:  # walkArray for direct lrt travel
        walkArray = [cleanDist, cleanTime, way1, way2]
    elif countway == 3:  # walkArrat for lrt travel involving interchanging stn in between
        walkArray = [cleanDist, cleanTime, way1, way2, way3]
    pgmap.save('templates/gui_frontend.html')
    return walkArray  # will be used for gui purposes


# ----------------------------------------------------------------------------------------------------------------------


def driving(src1, des1):
    pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)
    sitMarker(pgmap)
    startpoint1 = ox.geocode(src1)
    endpoint1 = ox.geocode(des1)

    startmarker1 = fol.Marker(location=startpoint1, popup='<strong>' + src1 + '</strong>',
                              tooltip="Start", icon=fol.Icon(color='blue', icon='car', prefix='fa'))
    startmarker1.add_to(pgmap)
    endmarker1 = fol.Marker(location=endpoint1, popup='<strong>' + des1 + '</strong>',
                            tooltip="End", icon=fol.Icon(color='red', icon='car', prefix='fa'))
    endmarker1.add_to(pgmap)
    start_node1 = str(ox.get_nearest_node(driveGraph, startpoint1, method='haversine'))
    end_node1 = str(ox.get_nearest_node(driveGraph, endpoint1, method='haversine'))
    # print("-----------------------------------------")
    ll = calculateShortest(start_node1, end_node1, "driving")
    # print(ll)
    distDrive = 0
    for m in range(len(ll) - 1):
        distDrive = distDrive + calculateDist(float(ll[m][0]), float(ll[m][1]), float(ll[m + 1][0]),
                                              float(ll[m + 1][1]))
    timeDrive = distDrive / 50 * 60
    cleanDist = str(round(distDrive, 2))
    cleanTime = str(round(timeDrive, 2))
    # print(cleanDist)
    # print(cleanTime)
    driveArray = [cleanDist, cleanTime]
    # DRIVE ROUTE
    fol.PolyLine(ll, color='red', weight=2.5, opacity=1).add_to(pgmap)
    pgmap.save('templates/gui_frontend.html')
    return driveArray


def lrt(st, en):
    # used for normal lrt routing(just lrt involved no walking)
    pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)
    sitMarker(pgmap)
    st1 = [st[0], st[1]]
    en1 = [en[0], en[1]]
    tt = calculateShortest(st1, en1, "lrt")
    pinning = []
    for x in pintag:  # check if exists in unique_list or not
        if x not in pinning:
            pinning.append(x)
    for i in range(len(pinning)):
        pinning[i].add_to(pgmap)

    fol.PolyLine(tt, color="black", weight=2.5, opacity=1).add_to(pgmap)


# ----------------------------------------------------- WALK ROUTING ---------------------------------------------------
def walkingBackEnd(startInput, endInput):
    walkingSrc = startInput
    walkingDes = endInput
    try:
        return walking(walkingSrc, walkingDes)
    except IndexError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        return "Routing out address is out of the range of the project scope"
    except ValueError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        return "Locations cannot be the same! Please try a different location for Start and End Point"


# ----------------------------------------------------- BUS ROUTING ----------------------------------------------------
def drivingBackEnd(startInput, endInput):
    drivingSrc = startInput
    drivingDes = endInput
    try:
        return driving(drivingSrc, drivingDes)
    except IndexError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        sitMarker(pgmap)
        return "Routing out address is out of the range of the project scope"
    except ValueError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        sitMarker(pgmap)
        return "Locations cannot be the same! Please try a different location for Start and End Point"

# -------------------------------------------- WALKING PLUS BUS ROUTING ------------------------------------------------
def walkPlusBusBackEnd(startInput, endInput):
    walkPlusBusSrc = startInput
    walkPlusBusEnd = endInput
    if walkPlusBusSrc == walkPlusBusEnd:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        sitMarker(pgmap)
        pgmap.save('templates/gui_frontend.html')
        return "Locations cannot be the same! Please try a different location for Start and End Point"
    try:
        data = walkPlusBus(walkPlusBusSrc, walkPlusBusEnd)
        buslist = data[0][0]
        noofstops = data[0][1]
        dis = data[0][2]
        distance = dis[0]
        time = dis[1]
        walkstmtstrt = data[1]
        walkstmtend = data[2]

        walktobusDist = data[3][0]
        walktobusTime = data[3][1]
        bustoendDist = data[4][0]
        bustoendTime = data[4][1]

        totalDist = str(round((float(distance) + float(walktobusDist) + float(bustoendDist)), 2))
        totalTime = str(round((float(time) + float(walktobusTime) + float(bustoendTime)), 2))

        return buslist, noofstops, walkstmtstrt, walkstmtend, totalDist, totalTime
    except IndexError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        sitMarker(pgmap)
        return "Routing out address is out of the range of the project scope"
    except ValueError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        sitMarker(pgmap)
        return "Locations cannot be the same! Please try a different location for Start and End Point"


# --------------------------------------------------- LRT ROUTING ------------------------------------------------------

def lrtBackEnd(startInput, endInput):
    walkingSrc = startInput
    walkingDes = endInput
    # start with walking so call the walkforlrt function and then it will carry on
    if walkingSrc == walkingDes:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        sitMarker(pgmap)
        pgmap.save('templates/gui_frontend.html')
        return "Locations cannot be the same! Please try a different location for Start and End Point"
    try:
        return walkforlrt(walkingSrc, walkingDes)
    except IndexError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        return "Routing out address is out of the range of the project scope"
    except ValueError:
        pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15,
                               truncate_by_edge=True)
        pgmap.save('templates/gui_frontend.html')
        return "Locations cannot be the same! Please try a different location for Start and End Point"

# ----------------------------------------------------------------------------------------------------------------------
