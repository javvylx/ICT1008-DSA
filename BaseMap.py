import networkx as nx
import osmnx as ox
import folium as fol
import os
import json
import requests
from IPython.display import IFrame

punggol = (1.403948, 103.909048)
G = ox.graph_from_point(punggol, distance=1000, truncate_by_edge=True, network_type="walk") # Network type there r different types of it, such as biking, driving, walking etc.
n, e = ox.graph_to_gdfs(G) # Convert a graph into node and/or edge GeoDataFrames
# n.to_csv('BuildingNodes.csv') # This converts edges in the map to coordinates in CSV format
# e.to_csv('Edges.csv') # This converts edges in the map to coordinates in CSV format

start = (103.9073345, 1.4060506) # Sample start coordinates
end = (103.9172982, 1.3956014) # Sample end coordinates

nodelist_G = list(G.nodes.values())

for i in range(0, len(nodelist_G)):
    if nodelist_G[i].get('y') == start[1] and nodelist_G[i].get('x') == start[0]:
        start_osmid = nodelist_G[i].get('osmid')
    if nodelist_G[i].get('y') == end[1] and nodelist_G[i].get('x') == end[0]:
        end_osmid = nodelist_G[i].get('osmid')

print(start_osmid, end_osmid) # Every Nodes in Open Street Map has an unique ID, this line will print the OSM ID of start and stop node and routing will be done


# Uncomment Line 29 and below to implement algorithm

# def routing(start_osmid, end_osmid):
#     pass  # Algorithm must be implemented here
#
# routedPath = routing(start_osmid, end_osmid) # Bind method to start and end node of OSMID
#
# ox.plot_graph_route(G, routedPath, fig_height=10, fig_width=10) # Plot out OSMNX routing on Matplotlib



