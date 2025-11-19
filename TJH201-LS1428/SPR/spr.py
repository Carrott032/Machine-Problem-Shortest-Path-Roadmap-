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
                    
            if (signOfPolygon > 0 and crossProduct < 0) or (signOfPolygon < 0 and crossProduct > 0):
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
                    # chek collisions 
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

    # Your code goes here. Note that for convenience, we 
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap. 
    
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