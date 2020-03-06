import networkx as nx
import osmnx as ox
import folium as fol
import os
import json
import requests
from IPython.display import IFrame

punggolNodes = open('BuildingNodes.json')
punggolEdges = open('Edges.json')

pgmap = fol.folium.Map(location=[1.403948, 103.909048], tiles='OpenStreetMap', zoom_start=12)



# G = ox.graph_from_point(punggol, distance=1000, truncate_by_edge=True, network_type="walk")
# # n, e = ox.graph_to_gdfs(G)
# # graph_map = ox.plot_graph_folium(G, popup_attribute='name', edge_width=2)

tooltip = "SIT New Punggol Campus"

logoIcon = fol.features.CustomIcon('siticon.png', icon_size=(40, 40))
fol.Marker(location=[1.413221, 103.911012], popup='<strong>SIT New Punggol Campus</strong>', tooltip=tooltip,
           icon=logoIcon).add_to(pgmap)
fol.GeoJson(punggolNodes, name="Punggol Nodes").add_to(pgmap)
fol.GeoJson(punggolEdges, name="Punggol Walking Routes").add_to(pgmap)

pgmap.save('penis.html')

# filepath = 'graph.html'
# graph_map.save(filepath)
# IFrame(filepath, width=600, height=500)
