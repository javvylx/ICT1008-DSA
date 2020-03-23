from flask import Flask, render_template, request
import folium as fol
import geopandas as gpd
import osmnx as ox


app = Flask(__name__)


@app.route('/')
def index():
    return render_template('user.html')

@app.route("/", methods=["GET", "POST"])
def need_input():
    for key, value in request.form.items():
        start = request.form["start"]
        end = request.form["end"]
        print("key: {0}, value: {1}".format(key, value))
    #print(start)
    #print(end)
    startLat, startLong = start.split(',', 1)
    endLat, endLong = end.split(',', 1)
    #print(startLat)
    #print(startLong)
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

    startmarker = fol.Marker(location=[startLat,startLong], popup='<strong>Start Point</strong>')
    startmarker.add_to(pgmap)
    endmarker = fol.Marker(location=[endLat,endLong], popup='<strong>End Point</strong>')
    endmarker.add_to(pgmap)
    pgmap.save('templates/gui_frontend.html')
    return render_template('user.html', start=start, end=end)

@app.route("/form", methods=["GET"])
def get_form():
    return render_template('user.html')

if __name__ == '__main__':
    app.run(debug=True)
