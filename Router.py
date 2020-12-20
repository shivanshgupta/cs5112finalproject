import re, string, sys 
import xml.sax as sax
import networkx as nx
import time
import random
import simplekml
from math import radians, cos, sin, asin, sqrt

USE_TIME = False

class node():
    def __init__(self, id, lat, lon):
        self.id = id
        self.lat = lat
        self.lon = lon


class way():
    def __init__(self, id):
        self.id = id
        self.nodes = []
        self.tags = {}

    def addNode(self, nd):
        self.nodes.append(nd)

    def addTag(self, k, v):
       self.tags[k] = v


class DictHandler(sax.handler.ContentHandler):
    def __init__(self):
        self.is_way = False
        self.nodes = []
        self.ways = []

    def startElement(self, name, attrs): 
        if name == "node":
            n = node(attrs["id"], attrs["lat"], attrs["lon"])
            self.nodes.append(n)

        if name == "way":
            self.is_way = True
            w = way(attrs["id"])
            self.ways.append(w)

        if name == "nd" and self.is_way == True:
            n = getNode(self.nodes, attrs["ref"])
            self.ways[-1].addNode(n)

        if name == "tag" and self.is_way == True:
            self.ways[-1].addTag(attrs["k"], attrs["v"])

    def endElement(self, name):
        if name == "way":
            self.is_way = False


"""
Return node object in nodes with id node_id
"""
def getNode(nodes, node_id):
    for n in nodes:
        if n.id == node_id:
            return n

    return None


"""
Calculate the great circle distance between two points 
on the earth (specified in decimal degrees)

Reference:
http://stackoverflow.com/questions/4913349/haversine-
formula-in-python-bearing-and-distance-between-two-gps-points
"""
def calcDistance(nd1, nd2):
    lat1 = float(nd1.lat)
    lon1 = float(nd1.lon)
    lat2 = float(nd2.lat)
    lon2 = float(nd2.lon)
      
    # Convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # Haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 

    # Radius of earth = 6367 km
    km = 6367 * c
      
    return km / 1.6


"""
Calculate the time taken to go from nd1 to nd2 using existing
speed limit information or make an assumtion based on area or
type of road

Reference: XXX
"""
def calcTime(nd1, nd2):
    if sys.argv[1] == "ITHACA": speed_limit = 30 # miles per hour
    elif sys.argv[1] == "NYC": speed_limit = 25 # miles per hour
    elif sys.argv[1] == "AZ": speed_limit = 55 # miles per hour
    else: speed_limit = 60 # miles per hour

    speed_limit = speed_limit / 3600 # miles per second

    return calcDistance(nd1, nd2) / speed_limit


"""
Returns the node_id of the node which is closest to the given latitude
and longitude

Reference:
https://stackoverflow.com/questions/41336756/find-the-closest-latitude
-and-longitude
"""
# def getClosest(nodes, lat, long):
#     return min(nodes, key=lambda p: calcDistance(v['lat'],v['lon'],p['lat'],p['lon']))


"""
Convert OSM file located at filepath to graph
"""
def createGraph(filepath):
    file = open(filepath, "r")
    
    handler = DictHandler()
    parser = sax.make_parser() 
    parser.setContentHandler(handler)
    parser.parse(file)

    nodes = handler.nodes
    ways = handler.ways


    # Create Graph
    G = nx.Graph()
    
    for w in ways:
        nds = w.nodes
        if len(nds) < 2: continue

        for i in range(len(nds) - 1):
            if USE_TIME: weight = calcTime(nds[i], nds[i+1])
            else: weight = calcDistance(nds[i], nds[i+1])
            G.add_edge(nds[i].id, nds[i+1].id, weight=weight)

    return G, nodes, ways 


"""
Creaes KML file associated with a given path
"""
def createKML(path, nodes, alg):
    kml = simplekml.Kml()
    coords = []

    path_lst = path.split(" -> ")

    for n in path_lst:
        node = getNode(nodes, n)
        coords.append([node.lon, node.lat])

    linestring = kml.newlinestring(name=alg + " Path")
    linestring.coords = coords
    kml.save('path' + alg + '.kml')

# Find the shortest path from source to destination in graph G
def dijkstra(G, source, destination):      
    return " -> ".join(nx.dijkstra_path(G, source, destination))

def astar(G, source, destination):      
    return " -> ".join(nx.astar_path(G, source, destination))

def bellman_ford(G, source, destination):
    return " -> ".join(nx.bellman_ford_path(G, source, destination))

def floyd_warshall(G, source, destination):
    predecessors, distance = nx.floyd_warshall_predecessor_and_distance(G)
    return " -> ".join(nx.reconstruct_path(source, destination, predecessors))

def johnson(G, source, destination):
    return " -> ".join(nx.johnson(G)[source][destination])


# List the nodes explored in order when finding the shortest path from 
# source to destination in graph G
def dijkstra_explored(G, source, destination):      
    return " -> ".join(nx.multi_source_dijkstra(G, source, target=destination, cutoff=None, weight="weight")[1])

def astar_explored(G, source, destination):
    path, explored = nx.astar_path(G, source, destination)
    visited = {}
    visited[destination] = path
    for curnode in explored.keys():
        path = [curnode]
        node = explored[curnode]
        while node is not None:
            path.append(node)
            node = explored[node]
        visited[curnode] = path[::-1] 
    return " -> ".join(visited)

def bellman_ford_explored(G, source, destination):
    return " -> ".join(nx.single_source_bellman_ford(G, source)[1])

def floyd_warshall_explored(G, source, destination):
    predecessors, distance = nx.floyd_warshall_predecessor_and_distance(G)
    visited = {}
    for key in predecessors.keys():
        visited[key] = nx.reconstruct_path(source, key, predecessors)
    return " -> ".join(visited)

def johnson_explored(G, source, destination):
    return " -> ".join(nx.johnson(G)[source])



if __name__ == '__main__':

    if sys.argv[1] == "TEST":
        data = "testData.osm"
        source = "129373"
        target = "129378"
    elif sys.argv[1] == "EXAMPLE":
        data = "exampleData.osm"
        source = "1554790199"
        target = "280095678"
    elif sys.argv[1] == "ITHACA":
        data = "ithacaData.osm"
        source = "213437176"
        target = "213445955"
        print("Source: 42.4416140, -76.4852500")
        print("Destination: 42.4393240, -76.5061340")
    elif sys.argv[1] == "NYC":
        data = "nycData.osm"
        source = "42446701"
        target = "42445920"
        print("Source: 40.7477730, -73.9850460")
        print("Destination: 40.7541000, -73.9783320")
    elif sys.argv[1] == "AZ":
        data = "azData.osm"
        source = "174878448"
        target = "174885440"
        print("Source: 34.0862039, -114.3302501")
        print("Destination: 34.0933730, -114.3521700")
    else:
        exit("Invalid test case")
    print("")

    
    G, nodes, ways = createGraph(data)


    start = time.time()
    path = dijkstra(G, source, target) 
    end = time.time()
    print("Dijkstra shortest path: " + path)
    print("Dijkstra runtime: " + str(end - start) + " seconds")
    createKML(path, nodes, "Dijkstra")
    path_length = nx.dijkstra_path_length(G, source, target)
    if USE_TIME: print("Dijkstra path time: " + str(path_length) + " seconds")
    else: print("Dijkstra path distance: " + str(path_length) + " miles")
    # explored = dijkstra_explored(G, source, target) 
    # print("Dijkstra explored: " + explored)
    print("")

    start = time.time()
    path = astar(G, source, target) 
    end = time.time()
    print("A* shortest path: " + path)
    print("A* runtime: " + str(end - start) + " seconds")
    createKML(path, nodes, "AStar")
    path_length = nx.astar_path_length(G, source, target)
    if USE_TIME: print("A* path time: " + str(path_length) + " seconds")
    else: print("A* path distance: " + str(path_length) + " miles")
    # explored = astar_explored(G, source, target) 
    # print("A* explored: " + explored)
    print("")

    start = time.time()
    path = bellman_ford(G, source, target) 
    end = time.time()
    print("Bellman Ford shortest path: " + path)
    print("Bellman Ford runtime: " + str(end - start) + " seconds")
    createKML(path, nodes, "BellmanFord")
    path_length = nx.bellman_ford_path_length(G, source, target)
    if USE_TIME: print("Bellman Ford path time: " + str(path_length) + " seconds")
    else: print("Bellman Ford path distance: " + str(path_length) + " miles")
    # explored = bellman_ford_explored(G, source, target) 
    # print("Bellman Ford explored: " + explored)
    print("")

    start = time.time()
    path = floyd_warshall(G, source, target) 
    end = time.time()
    print("Floyd Warshall shortest path: " + path)
    print("Floyd Warshall runtime: " + str(end - start) + " seconds")
    createKML(path, nodes, "FloydWarshall")
    _, distance = nx.floyd_warshall_predecessor_and_distance(G)
    path_length = distance[source][target]
    if USE_TIME: print("Floyd Warshall path time: " + str(path_length) + " seconds")
    else: print("Floyd Warshall path distance: " + str(path_length) + " miles")
    # explored = floyd_warshall_explored(G, source, target) 
    # print("Floyd Warshall explored: " + explored)
    print("")

    start = time.time()
    path = johnson(G, source, target) 
    end = time.time()
    print("Johnson shortest path: " + path)
    print("Johnson runtime: " + str(end - start) + " seconds")
    createKML(path, nodes, "Johnson")
    path_length = nx.dijkstra_path_length(G, source, target)
    if USE_TIME: print("Johnson path time: " + str(path_length) + " seconds")
    else: print("Johnson path distance: " + str(path_length) + " miles")
    # explored = johnson_explored(G, source, target) 
    # print("Johnson explored: " + explored)
    print("")