import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle
from shapely.geometry import LineString


class PRM:

    def __init__(self, mapDimensions):

        self.mapw, self.maph = mapDimensions
        self.listOfNodes = []
        self.listOfRoads = []

        plt.xlim([0, self.mapw])
        plt.ylim([0, self.maph])

    def randomPoint(self):
        x = int(random.uniform(1, self.mapw - 1))
        y = int(random.uniform(1, self.maph - 1))
        return x, y

    def drawNode(self, x, y, flag):                  # flag is given to identify start and end and draw them differently
        if flag == 0:
            node = Circle((x, y), 1, color = 'r')
        else:
            node = Circle((x, y), 0.5, color = 'g')
        plt.gca().add_patch(node)

    def distList(self, x, y):
        listOfNodes = self.listOfNodes.copy()
        listOfDist = []
        while listOfNodes != []:
            xn, yn = listOfNodes.pop(0)
            listOfDist.append(self.distance(xn, yn, x, y))
        listOfDist.sort()
        listOfDist.pop(0) #Remove the point itself
        return listOfDist

    def distance(self, x1, y1, x2, y2):
        dist = ((x2 - x1)**2+(y2-y1)**2)**0.5
        return dist

    def atDist(self, d, x, y):
        listOfNodes = self.listOfNodes.copy()
        while listOfNodes != []:
            xn, yn = listOfNodes.pop(0)
            if self.distance(xn, yn, x, y)==d:
                return xn, yn   
                
    def doesNotIntersect(self, xn, yn, x, y):
        lineA = LineString( [(xn, yn), (x, y)])
        listOfRoads = self.listOfRoads.copy()
        while listOfRoads != []:
            x1, y1, x2, y2 = listOfRoads.pop(0)
            x1, y1, x2, y2 = self.ignoreEnds(x1, y1, x2, y2)
            lineB = LineString([(x1, y1), (x2, y2)])
            if lineA.intersects(lineB):
                return False
        return True

    def addToList(self, xn, yn, x, y):
        self.listOfRoads.append((xn, yn, x, y))

    def drawLine(self, xn, yn, x, y):
        plt.plot([x, xn], [y, yn], color='#99E6FF')

    def ignoreEnds(self, x1, y1, x2, y2):
        k=100
        x_near_p1 = (k*x1 + x2)/(k+1)
        y_near_p1 = (k*y1 + y2)/(k+1)
        x_near_p2 = (k*x2 + x1)/(k+1)
        y_near_p2 = (k*y2 + y1)/(k+1)
        return x_near_p1, y_near_p1, x_near_p2, y_near_p2 # returning a nested list of coordinates

    def find_index(self,x,y):
        l = len(self.listOfNodes)
        for i in range(l):
            if x==self.listOfNodes[i][0] and y==self.listOfNodes[i][1]:
                return i

    def not_in_index_list(self,index,list_of_index):
        l = len(list_of_index)
        for i in range(l):
            if index == list_of_index[i]:
                return False
        return True

    def list_of_connected_nodes(self,x,y):
        listofroads = self.listOfRoads.copy()
        list_of_index = []
        l = len(listofroads)
        for i in range(l):
            if x == listofroads[i][0] and y==listofroads[i][1]:
                xn,yn = listofroads[i][2] , listofroads[i][3]
                index = self.find_index(xn,yn)
                if self.not_in_index_list(index,list_of_index):
                    list_of_index.append(index)
            if x == listofroads[i][2] and y==listofroads[i][3]:
                xn,yn = listofroads[i][0] , listofroads[i][1]
                index = self.find_index(xn,yn)
                if self.not_in_index_list(index,list_of_index):
                    list_of_index.append(index)
            
        return list_of_index

    def make_list_of_paths(self):
        list_of_paths = []
        list_of_nodes = self.listOfNodes.copy()
        l = len(list_of_nodes)
        for i in range(l):
            x,y = list_of_nodes.pop(0)
            list_of_connected_nodes = self.list_of_connected_nodes(x,y)
            list_of_paths.append(list_of_connected_nodes)
        return list_of_paths

class dijkstra: #operating as A*

    def __init__(self, n, listOfNodes, paths):

        self.n = n
        self.pathLen = []
        self.viaNode = []
        self.done = []
        self.all = []
        self.listOfNodes = listOfNodes
        self.paths = paths

        self.declarepathLen()
        self.declareviaNode()
        self.declareall()

    def declarepathLen(self):
        self.pathLen.append(0)
        for i in range(self.n - 1):
            self.pathLen.append(1000)

    def declareviaNode(self):
        for i in range(self.n):
            self.viaNode.append(0)

    def declareall(self):
        for i in range(self.n):
            self.all.append(i)

    def dist(self, a, b):
        x1, y1 = self.listOfNodes[a]
        x2, y2 = self.listOfNodes[b]
        dist = ((x2 - x1)**2+(y2-y1)**2)**0.5
        return dist

    def addToDoneList(self, a):
        self.done.append(a)

    def nextNode(self, c): #Use for Astar
        pathLen = self.pathLen.copy()
        done = self.done.copy()
        all = self.all.copy()
        left = list(set(all) - set(done))
        a = left.pop(0)
        min = pathLen[a] + self.dist(a, c)
        while left != []:
            b = left.pop(0)
            if pathLen[b] + self.dist(b, c) < min:
                a = b
                min = pathLen[a] + self.dist(a,c)
        return a

    def drawLine(self, a, b):
        x1, y1 = listOfNodes[a]
        x2, y2 = listOfNodes[b]
        plt.plot([x1, x2], [y1, y2], color = 'black')

k = 5
mapDimensions = (100, 100)
n = numOfNodes = 300
obsnum = 15
obsdim = 10

#PRM
ourPRM = PRM(mapDimensions)

c = n
while c != 0:  
    x, y = ourPRM.randomPoint()
    if True:
        if c==1 or c==n:             #Identify default start and end points
            ourPRM.drawNode(x, y,0)
        else:
            ourPRM.drawNode(x, y,1)
        ourPRM.listOfNodes.append((x, y))
        c = c-1

listOfNodes = ourPRM.listOfNodes.copy()
for i in range(n):
    x, y = listOfNodes.pop(0)
    distList = ourPRM.distList(x, y)

    for i in range(k):
        d = distList.pop(0)
        xn, yn = ourPRM.atDist(d, x, y)
        if ourPRM.doesNotIntersect(xn, yn, x, y):
            ourPRM.drawLine(xn, yn, x, y)
            ourPRM.addToList(xn, yn, x, y)


#A*
listOfNodes = ourPRM.listOfNodes.copy()
paths = ourPRM.make_list_of_paths()

dstra = dijkstra(numOfNodes, listOfNodes, paths)

a = 0
while a != n-1 :
    b = dstra.paths[a]
    b = list(set(b) - set(dstra.done))
    while b != []:
        c = b.pop(0)
        dist = dstra.dist(a, c) + dstra.pathLen[a] 
        if dist < dstra.pathLen[c]:
            dstra.pathLen[c] = dist
            dstra.viaNode[c] = a
    dstra.addToDoneList(a)
    a = dstra.nextNode(n-1)

a = n-1
while True:
    b = dstra.viaNode[a]
    dstra.drawLine(a, b)
    a = b
    if b == 0:
        break

plt.show()