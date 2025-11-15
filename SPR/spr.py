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

    for p in range (0, len(polygons)): #get to independed vetecies
        current = polygons[p]
        for i in range (0, len(current)):
            previous = current[(i - 1) % len(current)]
            current_vertex = current[i]
            next_vertex = current[(i + 1) % len(current)]
            
            #Solve for reflexive angle

            #create vector a and b
            ax = previous[0] - current_vertex[0]
            ay = previous[1] - current_vertex[1]
            bx = next_vertex[0] - current_vertex[0]
            by = next_vertex[1] - current_vertex[1]
            
            z = ax * by - ay * bx #Use Cross product to determine reflex angle
            if z > 0: #angle is reflexive
                vertices.append(current_vertex)

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
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    
    return path, pathLength

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
