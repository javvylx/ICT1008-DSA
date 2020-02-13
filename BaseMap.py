import networkx as nx
import osmnx as ox
from IPython.display import IFrame

location_point = (1.402208, 103.907128)
G = ox.graph_from_point(location_point, distance=1000, simplify=False)
graph_map = ox.plot_graph_folium(G, popup_attribute='name', edge_width=2)
fig, ax = ox.plot_graph(G, node_color='b', node_zorder=3)

filepath = 'graph.html'
graph_map.save(filepath)
IFrame(filepath, width=600, height=500)

origin_node = list(G.nodes())[0]
destination_node = list(G.nodes())[-1]
route = nx.shortest_path(G, origin_node, destination_node)

route_map = ox.plot_route_folium(G, route)

filepath = 'route.html'
route_map.save(filepath)
IFrame(filepath, width=600, height=500)

route_graph_map = ox.plot_route_folium(G, route, route_map=graph_map, popup_attribute='length')
filepath = 'route_graph.html'
route_graph_map.save(filepath)
IFrame(filepath, width=600, height=500)