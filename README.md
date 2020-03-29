# SIT ICT 1008 AY19/20 DSA Project
This is a Data Structure Algorithm project that is to develop a map application using Folium and OSMNX to guide the residents in Punggol to travel easily within one kilometre of Punggol area. 

There are currently two huge and crucial parts of the project:
- Finding the fastest route / shortest / least transfers route from one point to another point using data structure algorithms
- A web interface for users to interact with the whole project

## Getting Started
Clone the whole project folder as a ZIP file from put it into a offline folder of your choice:  
https://github.com/javvylx/1008-DSA/archive/master.zip


### Prerequisites
* [Anaconda Environment](https://www.anaconda.com/distribution/) - The all in one python package manager and deployment tool kit 
* [Python 3.7](https://www.python.org/downloads/release/python-370/) - Base Python Interpreter 
* [Pycharm Community](https://www.jetbrains.com/pycharm/download/download-thanks.html?platform=windows&code=PCC) - Python IDE for professional developer


### Installing
Make sure the above prerequisities are installed before continuing 

**Step 1:** Creation of anaconda python environment
Open Anaconda Navigator -> Navigate to enviroments  
-> Create -> Select Python package: 3.7

**Step 2:** Installation of packages/libraries
Click on the play button of the name of the environment  
-> Open with terminal  

Enter the following commands one after each package has completed installation:

```
conda install -c conda-forge folium
```
```
conda install -c conda-forge osmnx
```
```
conda install -c anaconda pillow
```
```
conda install -c anaconda flask
```
**Step 3:** Binding Conda Interpreter to Pycharm
Open Pycharm Community Edition -> Select "File" from top navigation bar -> Open the project folder  
-> Select the whole project folder
- Select "File" from top navigation bar -> "Settings" -> Go under the project folder and Select project interpreter -> Select "Show All..."
- Under the Project Interpreters window -> Select on the "+" icon 
- Select Conda Enviroment -> Select Existing Enviroment -> Select "..." on the right side of Interpreter
- Add the interpreter which is usually under: 
```
C:\Users\{{PC_Name}}\Anaconda3\envs\{{Name_Of_Enviroment}}\python.exe
```
- Apply all options and Select "Ok"

**Step 4:** Run **flask_control_start.py** 

Installation Guide with images:
https://imgur.com/a/IRoFsJq


## Built With
* [Folium](https://python-visualization.github.io/folium/) - Data visualization for maps
* [Osmnx](https://python-visualization.github.io/folium/) - Visualization for street networks 
* [OpenStreetMap](https://www.openstreetmap.org/) - Open Source API for mapping data 


## Authors

- Javier Lim
- Tan Yi Hao 
- Heng Pei Min
- Farin Tahrima Rahman
- Goh Jia En
- Teo Zhong Peng

## References 
OSMNX:
  * https://github.com/gboeing/osmnx-examples/tree/master/notebooks
  * https://osmnx.readthedocs.io/en/stable/osmnx.html
  * https://networkx.github.io/documentation/stable/index.html
  
Folium:
  * https://python-visualization.github.io/folium/modules.html
  * https://python-visualization.github.io/folium/quickstart.html
  
OpenStreetMap:
  * https://wiki.openstreetmap.org/wiki/Map_Features
  * https://wiki.openstreetmap.org/wiki/Overpass_API/Language_Guide
  * https://www.mytransport.sg/content/mytransport/home/dataMall/dynamic-data.html
  
Opensource APIs used:
  * https://www.mytransport.sg/content/mytransport/home/dataMall.html
  * https://www.mytransport.sg/content/mytransport/home/dataMall/dynamic-data.html
  * https://wiki.openstreetmap.org/wiki/Map_Features
  * https://wiki.openstreetmap.org/wiki/Overpass_API/Language_Guide
  
Basemap Plotting:
  * http://geojson.io/
  * https://pypi.org/project/reverse-geocode/
  * https://geopandas.org/
  
Algorithm Used:
  * https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
  * https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/
  * https://www.geeksforgeeks.org/a-search-algorithm/
  
