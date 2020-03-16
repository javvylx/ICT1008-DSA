import networkx as nx
import osmnx as ox
import folium as fol
import geopandas as gpd
import json
import heapq as hq
import math
import os
import json
import requests
from IPython.display import IFrame

punggol = gpd.read_file('map.geojson')
polygon = punggol['geometry'].iloc[0]
lrtGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, infrastructure='way["railway"]')
walkGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="walk")
Buildings = ox.footprints.footprints_from_polygon(polygon, footprint_type='building', retain_invalid=False)
driveGraph = ox.core.graph_from_polygon(polygon, truncate_by_edge=True, network_type="drive")
penis = ox.project_gdf(Buildings)

walkNode, walkEdge = ox.graph_to_gdfs(walkGraph)  # Convert a graph into node and/or edge GeoDataFrames
driveNode, driveEdge = ox.graph_to_gdfs(driveGraph)
lrtNode, lrtEdge = ox.graph_to_gdfs(lrtGraph)

pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='openstreetmap', zoom_start=15, truncate_by_edge=True)

driveGraphL = fol.GeoJson(driveEdge, name="Public Roads")
driveGraphL.add_to(pgmap)

walkEdgeL = fol.GeoJson(walkEdge, name="Walking Paths")
walkEdgeL.add_to(pgmap)

lrtGraphL = fol.GeoJson(lrtEdge, name="LRT Track")
lrtGraphL.add_to(pgmap)

fol.GeoJson(penis, name='Buildings').add_to(pgmap)
fol.LayerControl(collapsed=True).add_to(pgmap)

logoIcon = fol.features.CustomIcon('siticon.png', icon_size=(40, 40))
SITtooltip = fol.Marker(location=[1.413006, 103.913249], popup='<strong>SIT New Punggol Campus</strong>', icon=logoIcon)
SITtooltip.add_to(pgmap)

# # def convertToCoord(path, n):
# #     route = []
# #     for id in path:
# #         route.append([n[id][0]['lat'], n[id][0]['lon']])
# #
# #     return route
# #     # return a list of coordinates to plot
#
# # A* algorithm (priority)f(s) = (cost)g(s) + h(s)(estimation of remaining cost - heuristic)
# def heuristic(x1, y1, x2, y2):
#     # euclidean distance heuristic but omit square root to improve performance.
#     # non-admissible as distance could be more than the heuristic of original start to end
#     return math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2)
#
# def dijkstra(start, end, edgesdict, nodesdict):
#     heap = [(0, start, [])]  # cost, current node, path
#     hq.heapify(heap)  # heapify the list (Just to make sure)
#     visited = []  # visited nodes
#     dist = {start: 0}  # initialize all other nodes distance to infinity
#     while True:
#         total_dist, cur_vertex, cur_path = hq.heappop(heap)
#         # check if current vertex has been visited
#         if cur_vertex in visited or cur_vertex not in edgesdict:
#             continue
#         # else append into visited list
#         visited.append(cur_vertex)
#         new_path = cur_path + [cur_vertex]
#         # return condition
#         if cur_vertex == end:
#             return cur_path
#         for node, cur_dist in edgesdict[cur_vertex]:
#             if node not in dist or total_dist + cur_dist < dist[node]:
#                 dist[node] = cur_dist + total_dist
#                 priority = cur_dist + total_dist + heuristic(nodesdict[node][0]['lat'], nodesdict[node][0]['lon'],
#                                                              nodesdict[end][0]['lat'], nodesdict[node][0]['lon'])
#                 hq.heappush(heap, (priority, node, new_path))
#
# def calculateShortest(self, start_node, end_node):
#     path = dijkstra(start_node, end_node, self.e, self.n)
#     ox.plot_route_folium(pgmap, path, popup_attribute='length')
#
# start_node = [103.9073345, 1.4060506]  # Sample start coordinates
# end_node = [103.9172982, 1.3956014]  # Sample end coordinates
# calculateShortest(start_node, end_node)

pgmap.save('templates/gui_frontend.html')
