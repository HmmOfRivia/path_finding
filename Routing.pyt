import arcpy
import heapq
import os
from math import sqrt
import numpy as np
from timeit import default_timer as timer
from ast import literal_eval

def euclideanDistance(x1,y1,x2,y2):
    '''
    Computes euclidean distance between two points

    Parameters
    ----------
    x1: float
    	X coordinate of the first point
    y1: float
	Y coordinate of the first point
    x2: float
	X coordinate of the second point
    y2 float:
    	Y coordinate of the second point

    Returns
    -------
    float
    	Euclidean distance between two points
    '''
    return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

class Node:
    '''
    Representation of nodes - part of graph
    
    Parameters
    ----------
    x: float
        X coordinate of the node
    y: float
        Y coordinate of the node
        
    Attributes
    ----------
    X: float
    	Storage of X coordinate of the node
    Y: float
    	Storage of Y coordinate of the node
    Edges: list
        List containing edges to neighbours
    '''
    def __init__(self, x, y):
        self.Id = (x, y)  
        self.X = x
        self.Y = y
        self.Edges = []  


class Edge: 
    '''
    Representation of edges - part of graph
    
    Parameters
    ----------
    fid:
    	Unique id from shp files
    from: Node
    	Node that, the edge is coming from
    To: Node
        Node that, the edge is ending in
    cost: float
        Length of the edge (distance between from and to nodes) [meters]
    type: float
        Type of road
    
    Attributes
    ----------
    Id:
    	Unique id of the edge
    From: 
    	Starting node of the edge
    To:
        Ending node of the edge
    Cost:
        Length of the edge (distance between from and to nodes) [meters]
    Type:
        Time needed to finish segement of the road [minutes] 
    '''	
    def __init__(self, fid, f, t, c, y):
        self.Id = fid
        self.From = f
        self.To = t
        self.Cost = c
        if y == "SKDR01":
            self.Type = (self.Cost / 2380)  # 140 km/h
        elif y == "SKDR02":
            self.Type = (self.Cost / 2040)  # 120 km/h
        elif y == "SKDR03":
            self.Type = (self.Cost / 1700)  # 100 km/h
        elif y == "SKDR04":
            self.Type = (self.Cost / 1360)  # 80 km/h
        elif y == "SKDR05":
            self.Type = (self.Cost / 850)  # 50 km/h
        elif y == "SKDR06":
            self.Type = (self.Cost / 850)  # 50 km/h
        elif y == "SKDR07":
            self.Type = (self.Cost / 510)  # 30 km/h
        elif y == "SKDR08":
            self.Type = (self.Cost / 340)  # 20 km/h

class Graph:
    '''
    Representation of graph as object.
    
    Attributes
    ----------
    Edges: Dictionary
    	Dictionary containg edges of the graph. Given in (nodeFrom, nodeTo):edge key:value format.
    Nodes: Dictionary
        Dictionary contating nodes of the graph. Given in (x,y):node format.
    '''  
    def __init__(self):
        self.Edges = {}  
        self.Nodes = {}  

    def AddEdge(self, edge):
        '''
        Adding edge to the graph. Also adding Node from and node To Nodes.
        
        Parameters
    	----------
    	edge: Edge
    		Edge to be added
        '''
        self.Edges[(edge.From.Id, edge.To.Id)] = edge
        self.Nodes[edge.From.Id].Edges.append(self.Edges[(edge.From.Id, edge.To.Id)])
        self.Nodes[edge.To.Id].Edges.append(self.Edges[(edge.From.Id, edge.To.Id)])

    def AddNode(self, node):
        '''
        Adding node to the graph.
        
        Parameters
    	----------
    	enode: Nodee
    		Node to be added
        '''
        self.Nodes[node.Id] = node

def Dijkstra_Heapq(graph, startNode, finishNode, mode):
    '''
    Computing the shortest/fastest path between two points using Dijkstra algorithm.
    
    Parameters
    ----------
    graph: Graph
    	Graph representing road network in which the shortest/fastest path will be computed.
    startNode: Node
        Point from which path is computed
    finishNode: Node
        Destination point to compute path
    mode: string:
        Determines which path is to be computed. Possible options are the shortest and the fastest
	
    Returns
    -------
    Dictionary
        Path between points in node:edge format.
    '''
    start = timer()
    if (startNode in graph.Nodes) and (finishNode in graph.Nodes):
        p = {x: -1 for x in graph.Nodes}
        distances = {startNode: 0}
        min_dist = [(0, startNode)]
        visited = []
        path = []
        counter = 0

        while min_dist:
            current_weight, cur = heapq.heappop(min_dist)
            visited.append(cur)

            for v in range(len(graph.Nodes[cur].Edges)):
                neighbourline = graph.Nodes[cur].Edges[v]

                neighbour = graph.Nodes[cur].Edges[v].To
                if neighbour.Id == cur:
                    neighbour = graph.Nodes[cur].Edges[v].From
		if mode == "short":
                	this_dist = current_weight + graph.Nodes[cur].Edges[v].Cost
		elif mode == "fast":
			this_dist = current_weight + graph.Nodes[cur].Edges[v].Type

                if neighbour.Id not in distances or this_dist < distances[neighbour.Id]:
                    distances[neighbour.Id] = this_dist
                    heapq.heappush(min_dist, (this_dist, neighbour.Id))
                    p[neighbour.Id] = (cur, neighbourline.Id)
		    counter += 1
    if mode == "short":
        arcpy.AddMessage("Shortest distance between points: " + str(distances[finishNode]))
    if mode == "fast":
        arcpy.AddMessage("Fastest road will take "+ str(distances[finishNode]) + ' min')
    end = timer()
    arcpy.AddMessage("Computed distances to " + str(counter) + " points")
    arcpy.AddMessage("Algorithm was running for: " + str(end-start) + " [s]")
    return p

def AStar(graph, startNode, endNode, mode):
    '''
    Computing the shortest/fastest path between two points using A* algorithm.
	
    Parameters
    ----------
    graph: Graph
    	Graph representing road network in which the shortest/fastest path will be computed.
	startNode: Node
		Point from which path is computed
	finishNode: Node
		 Destination point to compute path
	mode: string:
		 Determines which path is to be computed. Possible options are the shortest and the fastest
	
    Returns
    -------
    Dictionary
    Path between points in node:edge format.
    '''

    start = timer()
    if (startNode in graph.Nodes) and (endNode in graph.Nodes):
        
    	distance = {startNode:0} # distance from startNode to current node
    	costFunction = {node: 50000 for node in graph.Nodes}
    	path = {x: -1 for x in graph.Nodes}  # latest visited node
    	S = []  # List of visited nodes
    	heuristics= {endNode:0}
        S.append(startNode)
	counter = 0
        Q=[(0, startNode)]
        ProcessedNodes = 1
        while Q:
            cost, nodeWithLowestCost = heapq.heappop(Q)  # node with lowest value of costFunction is next node of route computing
            currentNode = nodeWithLowestCost
            if currentNode == endNode:
                end = timer()
                S.append(currentNode)
                arcpy.AddMessage("Computed distances to " + str(ProcessedNodes) + " points")
    		arcpy.AddMessage("Algorithm was running for: " + str(end-start) + " [s]")
                return path

            S.append(currentNode)

            for v in range(len(graph.Nodes[currentNode].Edges)):  # get every neigbours of node

                neighbourEdge = graph.Nodes[currentNode].Edges[v]  # edge for one neighbour

                neighbour = neighbourEdge.To

                if neighbour.Id == currentNode:
                    neighbour = neighbourEdge.From

                if neighbour.Id in S:
                    continue
		if mode == 'short':

                	heuristics[neighbour.Id] = abs(endNode[0] - neighbour.Id[0]) + abs(endNode[1] - neighbour.Id[1])
			currentDist = cost + neighbourEdge.Cost
		elif mode == 'fast':
			heuristics[neighbour.Id] = abs((abs(endNode[0] - neighbour.Id[0]) + abs(endNode[1] - neighbour.Id[1])) / 2380)
			currentDist = cost + neighbourEdge.Type

                if neighbour.Id not in distance or currentDist < distance[neighbour.Id]:
    			distance[neighbour.Id] = currentDist
    			path[neighbour.Id] = (currentNode, neighbourEdge.Id)
    			counter +=1
			costFunction[neighbour.Id] = distance[neighbour.Id] + heuristics[neighbour.Id]
                        heapq.heappush(Q,(costFunction[neighbour.Id], neighbour.Id))
                ProcessedNodes += 1
    end = timer()
    return path


def cond(graph, aim, path):
    '''
    Selecting all elements of path.
    
    Parameters
    ----------
    graph: Graph
        
    aim: Coordinates of end point
        
    path: Dictionary with last visited nodes
    '''

    condition = ''
    while path[aim] != -1:  # value of p dict is a tuple where 0 is number of current node, 1 - fid of edge
        condition = condition + str(path[aim][1]) + ','
        aim = path[aim][0]
    condition = '"FID" IN (' + condition.rstrip(',') + ')'
    return condition

def eliminateOuterNodes(graph, startNode):
    '''
    Function to eliminate nodes that are not part of the graph
    
    Parameters
    ----------
    graph: Graph
    	Graph to be searched in order to get nodes that are not part of the graph.
    startNode: Node
    	Point in center of graph
	
    Returns
    -------
    Dictionary
    	Path between points in node:edge format.
    '''
    p = {x:-1 for x in graph.Nodes}
    dist = {}
    node = [(startNode)]
    while node:
        cur = heapq.heappop(node)
        for v in range(len(graph.Nodes[cur].Edges)):
            neighbour = graph.Nodes[cur].Edges[v].To
            if neighbour.Id == cur:
                neighbour = graph.Nodes[cur].Edges[v].From
            if neighbour.Id not in dist:
                dist[neighbour.Id]=0
                heapq.heappush(node, (neighbour.Id))
                p[neighbour.Id] = 1
    return p

def findNode(graph, x, y, path):
    '''
    Finding closest point to given coordinates.
    
    Parameters
    ----------
    graph: Graph
    	Graph to be searched in order to find closest point.
    x: float 
        X coordinate of point
    y: float 
        Y coordinate of point	
    path: Dictionary
    	
    
    Returns
    -------
    Node
    	Closest point to given coordinates in (x,y) format
    '''

    nodes = graph.Nodes.keys()
    distances = [(0,0),0]
    pom = 0

    for v in nodes:
        distance = math.sqrt((v[0]-x)**2+(v[1]-y)**2)

        if (distance <= distances[1] or pom == 0) and (path[v] == 1):
            distances[1] = distance
            distances[0] = v
            pom = 1
    return distances[0]

def notFound(path, nodesAmount):
    '''
    Determines how many nodes are not connected with graph.

    Parameters
    ----------
    path:
    	
    nodesAmount:
    	Total number of nodes 
    '''
    notFound = 0
    for v in path:
    	if path[v] == -1:
    		notFound = notFound + 1
        else:
            	None
    arcpy.AddMessage(str(notFound) + '/' + str(nodesAmount) + ' nodes are not connected with graph')

class Toolbox(object):
    def __init__(self):
        self.label = "Routing"
        self.alias = "routing"

        # List of tool classes associated with this toolbox
        self.tools = [RoutingD]


class RoutingD(object):
    # adds geometry attributes needed for graph
    def __init__(self):
        self.label = "Routing"
        self.description = "Computing the shortest/fastest path between given points on given road layer"

    def getParameterInfo(self):
        # Define parameter definitions

        # Input Features parameter
        inputRoads = arcpy.Parameter(
            displayName="Input roads layer",
            name="inputRoads",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        inputRoads.filter.list = ["Polyline"]

	#Mode parameter
	mode = arcpy.Parameter(
            displayName="Route type",
            name="mode",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        mode.filter.list = ['short', 'fast']
        mode.value = 'short'

	coordsFirst = arcpy.Parameter(
            displayName="Coordinations of start point:",
            name="coordsFirst",
            datatype="String",
            parameterType="Required",
            direction="Input")

	coordsLast = arcpy.Parameter(
            displayName="Coordinations of end point:",
            name="coordsLast",
            datatype="String",
            parameterType="Required",
            direction="Input")

	#Optimization parametr
	optimization = arcpy.Parameter(
            displayName="Choose Algorithm",
            name="optimalizaton",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        optimization.filter.list = ['aStarHeapQ', 'dijkstraHeapq']
        optimization.value = 'aStarHeapQ'

        # Derived Output Features parameter
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")

        outFeatures.parameterDependencies = [inputRoads.name, outFeatures.name]
        outFeatures.schema.clone = True

        parameters = [inputRoads, mode, optimization, coordsFirst, coordsLast, outFeatures]

        return parameters

    def isLicensed(self):  # optional
        return True

    def updateMessages(self, parameters):  # optional
        return

    def execute(self, parameters, messages):
        inFeatures = parameters[0].valueAsText
	routeType = parameters[1].valueAsText
        startValue = parameters[3].valueAsText
	endValue = parameters[4].valueAsText
	optimization = parameters[2].valueAsText
	outFeatures = "points.shp"
        arcpy.overwriteoutput = True

        arcpy.AddGeometryAttributes_management(inFeatures, ["LENGTH", "LINE_START_MID_END"])

        mainGraph = Graph()
        
        # loading data
	repeatingNodes = 0
        repeatingEdges = 0
	rowsInCursor = 0
        cursor = arcpy.SearchCursor(inFeatures)
        for row in cursor:
	    rowsInCursor = rowsInCursor + 1
            # arcpy.AddMessage(row.fid)
            # if there is no node in graph, add it
            start = Node(int(round(row.start_x)), int(round(row.start_y)))  # start point
            if not start.Id in mainGraph.Nodes:
                mainGraph.AddNode(start)
	    else:
                repeatingNodes = repeatingEdges + 1
            last = Node(int(round(row.end_x)), int(round(row.end_y)))  # end point
            if not last.Id in mainGraph.Nodes:
                mainGraph.AddNode(last)
	    else:
                repeatingNodes = repeatingEdges + 1
	    edge = Edge(row.fid, start, last, round(row.length), row.x_kod)
	    if not (start.Id, last.Id) in mainGraph.Edges:            
		mainGraph.AddEdge(edge) 
	    else:
                repeatingEdges = repeatingEdges + 1

	constantGraph = eliminateOuterNodes(mainGraph, (473030, 572013))

        

	beginPoint = literal_eval(startValue)
	endPoint = literal_eval(endValue)
	beginClosestNode = findNode(mainGraph, beginPoint[0], beginPoint[1], constantGraph)
	endClosestNode = findNode(mainGraph, endPoint[0], endPoint[1], constantGraph)
	if optimization == 'dijkstraHeapq':
            arcpy.AddMessage('Dijkstra stats:')
            a = Dijkstra_Heapq(mainGraph, beginClosestNode, endClosestNode, routeType)
        elif optimization == 'aStarHeapQ':
            arcpy.AddMessage('AStar stats:')
            a = AStar(mainGraph, beginClosestNode, endClosestNode, routeType)    
        
	
        condition =  cond(mainGraph, endClosestNode, a)
        
        arcpy.SelectLayerByAttribute_management(inFeatures, 'NEW_SELECTION',condition)

	filename = 'Route' #name of new file
        path = arcpy.env.workspace #path - geobase
        filepath = os.path.join(path, filename)

	arcpy.CopyFeatures_management(inFeatures, filepath) #copy to new shp

        arcpy.SelectLayerByAttribute_management(inFeatures, 'CLEAR_SELECTION') #clear selection

        #display
        mxd = arcpy.mapping.MapDocument('CURRENT')
        df = arcpy.mapping.ListDataFrames(mxd, 'Layers')[0]
        addLayer = arcpy.mapping.Layer(filepath)
        arcpy.mapping.AddLayer(df, addLayer, 'BOTTOM')
        arcpy.RefreshActiveView()
        arcpy.RefreshTOC()