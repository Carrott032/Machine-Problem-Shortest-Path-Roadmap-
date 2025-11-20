import sys
import numpy as np


'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices=[]
    
    # Your code goes here 
    # You should return a list of (x,y) values as lists, i.e.
    # vertices = [[x1,y1],[x2,y2],...]

    # Explanation:
    # Polygons is an array [[[x1, y1], [x2, y2], ...], [[x1, y1], [x2, y2], ...] ...]
    a = 0
    while(a < len(polygons)):
        currPolygon = polygons[a]
        i = 0
        while (i < len(currPolygon)):
            lastVertex = currPolygon[(i - 1) % len(currPolygon)] # We do the modulo to loop around
            currentVertex = currPolygon[i]
            nextVertex = currPolygon[(i + 1) % len(currPolygon)]
            
            #Vector 1 is from last vertex to current, vector 2 is from current to next
            vec1_x = currentVertex[0]-lastVertex[0]; vec1_y = currentVertex[1]-lastVertex[1]
            vec2_x = nextVertex[0] - currentVertex[0]; vec2_y = nextVertex[1] - currentVertex[1]
            crossProduct = vec1_x * vec2_y - vec1_y * vec2_x

            # If the cross product is less than 0, that means the signed area of the parallelogram the cross product creates is less than 0, which means the turn is CCW
            # If its positive its CW
            

            # Now we need to determine whether the polygon itself is CW or CCW so we can tell which angles are obtuse
            signOfPolygon = 0.0
            numVertices = len(currPolygon)
            for j in range(numVertices):
                x1, y1 = currPolygon[j]
                x2, y2 = currPolygon[(j + 1) % numVertices]
                signOfPolygon += x1 * y2 - x2 * y1
                    
            if not ( (signOfPolygon > 0 and crossProduct < 0) or (signOfPolygon < 0 and crossProduct > 0)):
                vertices.append(currPolygon[i])

            i += 1
        a += 1
    return vertices


'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = dict()
    
    # Your code goes here
    # You should check for each pair of vertices whether the
    # edge between them should belong to the shortest path
    # roadmap. 
    #
    # Your vertexMap should look like
    # {1: [5.2,6.7], 2: [9.2,2.3], ... }
    #
    # and your adjacencyListMap should look like
    # {1: [[2, 5.95], [3, 4.72]], 2: [[1, 5.95], [5,3.52]], ... }
    #
    # The vertex labels used here should start from 1

    # label reflexive verticies
    for i in range (0, len(reflexVertices)):
        vertexMap[i + 1] = reflexVertices[i]
        for j in range (0, len(reflexVertices)):
            if i != j:
                #check if legal edge btween reflexive vertices
                point1 = reflexVertices[i]
                point2 = reflexVertices[j]
                legalEdge = True

                for p in range (0, len(polygons)):
                    # check collisions 
                    currPolygon = polygons[p]

                    if point1 in currPolygon and point2 in currPolygon:
                        index1 = currPolygon.index(point1)
                        index2 = currPolygon.index(point2)
                        
                        # add each vertex in between points
                        n = len(currPolygon)
                        
                        forward = []
                        for k in range(1, (index2 - index1) % n):
                            forward.append((index1 + k) % n)
                        backward = []
                        for k in range(1, (index1 - index2) % n):
                            backward.append((index2 + k) % n)
                        
                        # check if any vertex in between is reflexive 
                        forward_blocked = any(currPolygon[i] in reflexVertices for i in forward)
                        backward_blocked = any(currPolygon[i] in reflexVertices for i in backward)
                        

                        #if none are reflexive then the edge is valid 
                        if forward_blocked and backward_blocked:
                            legalEdge = False
                            break
                        else:
                            continue

                    #break down to each edge of polygon
                    for k in range (0, len(currPolygon)):
                        nextK = (k + 1) % len(currPolygon)

                        # Skip if the points are right next to each other on polygon (They make a legal edge)
                        if (point1 == currPolygon[k] or point1 == currPolygon[nextK] or point2 == currPolygon[k] or point2 == currPolygon[nextK]):
                            continue
                        
                        #line intersection check 

                        # check parallel using cross product
                        cross = (point2[0] - point1[0]) * (currPolygon[nextK][1] - currPolygon[k][1])- (point2[1] - point1[1]) * (currPolygon[nextK][0] - currPolygon[k][0])
                        if cross == 0:
                            continue #parallel lines mean they cannot intersect
                        
                        # use parametric equations to solve for intersection
                        ua = ((currPolygon[nextK][0] - currPolygon[k][0]) * (point1[1] - currPolygon[k][1]) - (currPolygon[nextK][1] - currPolygon[k][1]) * (point1[0] - currPolygon[k][0])) / cross
                        ub = ((point2[0] - point1[0]) * (point1[1] - currPolygon[k][1]) - (point2[1] - point1[1]) * (point1[0] - currPolygon[k][0])) / cross

                        if ua >= 0 and ua <= 1 and ub >= 0 and ub <= 1:
                            legalEdge = False
                            break

                if legalEdge:
                    dist = np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

                    #create definition if not exists
                    if (i + 1) not in adjacencyListMap:
                        adjacencyListMap[i + 1] = []

                    adjacencyListMap[i + 1].append([j + 1, round(float(dist), 2)])
                
    return vertexMap, adjacencyListMap



'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    frontier = [(0.0, start)] # Array of (cost, vertex)
    parent = {start: None} # Dictionary for child -> parent
    cost = {start: 0.0} # Dictionary for cost
    visited = [] # List of all the visited nodes

    def leastCostIndex(open_list):
        bestIndex = 0
        for i in range(1, len(open_list)):
            if open_list[i][0] < open_list[bestIndex][0]:
                bestIndex = i
        return bestIndex



    while frontier:
        index = leastCostIndex(frontier)
        theCost, n = frontier.pop(index)

        if n in visited:
            continue # disregard if we already visited it

        if n == goal:
            reversePath = []
            p = n
            while p is not None:
                reversePath.append(p)
                p = parent[p]
            path = reversePath[::-1] 
            return path, theCost

        neighbors = adjListMap.get(n, [])
        for a in neighbors:
            v = a[0]
            w = a[1]
            newCost = theCost + w
            old = cost.get(v, None)
            if old is None or newCost < old:
                cost[v] = newCost
                parent[v] = n
                frontier.append((newCost, v))

        visited.append(n)


    return path, pathLength

    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    keys = list(adjListMap.keys())
    keyLen = len(keys)
    for i in range(keyLen):
        node = keys[i]
        neighborList = adjListMap[node]

        d = []
        for k in range(len(neighborList)):
            d.append([neighborList[k][0], neighborList[k][1]])

        updatedALMap[node] = d

    start = [x1, y1]
    goal  = [x2, y2]
    vertexMap[startLabel] = start
    vertexMap[goalLabel]  = goal

    def noCollisions(p1, p2):

        def segmentsIntersect(a, b, c, d): 
            x1, y1 = a
            x2, y2 = b
            x3, y3 = c
            x4, y4 = d
            
            if ((x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3)) == 0.0:
                return False
            if a == c or a == d or b == c or b == d:
                return False
            e = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3))
            f = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3))

            return 0.0 <= e <= 1.0 and 0.0 <= f <= 1.0

        for p in polygons:
            idx = 0
            while idx < len(p):
                if segmentsIntersect(p1, p2, p[idx], p[(idx + 1) % len(p)]):
                    return False
                idx += 1

        return True

    def addEdgeBetweenVertices(v1, v2, v1Index, v2Index):
        length = round(float(np.sqrt((v1[0] - v2[0])*(v1[0] - v2[0]) + (v1[1] - v2[1])*(v1[1] - v2[1]))), 2)
        if v1Index not in updatedALMap:
            updatedALMap[v1Index] = []
        if v2Index not in updatedALMap:
            updatedALMap[v2Index] = []
        updatedALMap[v1Index].append([v2Index, length])
        updatedALMap[v2Index].append([v1Index, length])



    keys2 = list(vertexMap.keys())
    i = 0
    while i < len(keys2):
        if keys2[i] != startLabel and keys2[i] != goalLabel:
            x1, x2 = goal
            if noCollisions(goal, vertexMap[keys2[i]]):
                addEdgeBetweenVertices(goal, vertexMap[keys2[i]], goalLabel, keys2[i])
            x1, x2 = start
            if noCollisions(start, vertexMap[keys2[i]]):
                addEdgeBetweenVertices(start, vertexMap[keys2[i]], startLabel, keys2[i])
        i += 1

    if noCollisions(start, goal): #we go straight to the end
        addEdgeBetweenVertices(start, goal, startLabel, goalLabel)

    return startLabel, goalLabel, updatedALMap

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append(list(map(float, xys[p].split(','))))
        polygons.append(polygon)

    
    # Print out the data
    print ("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print (str(polygons[p]))
    print ("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print ("Reflexive vertices:")
    print (str(reflexVertices))
    print ("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print ("Vertex map:")
    print (str(vertexMap))
    print ("")
    print ("Base roadmap:")
    print (str(adjListMap))
    print ("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print ("Updated roadmap:")
    print (str(updatedALMap))
    print ("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print ("Final path:")
    print (str(path))
    print ("Final path length:" + str(length))
    

    # Extra visualization elements goes here
