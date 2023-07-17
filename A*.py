import math
import heapq
from Node import Node
import time

def setupField():
    field = [None] * 2304
    for i in range(48):
        for j in range(48):
            if ((i >= 14 and i < 34) and ((j >= 0 and j < 10) or (j >= 38))): # Goals
                field[(i * 48) + j] = 1
            elif ((i >= 6 and i < 42) and (j >= 21 and j < 27)): # Center vertical bar
                field[(i * 48) + j] = 1
            elif ((i >= 5 and i < 11) or (i >= 37 and i < 43)) and ((j >= 13 and j < 35)): # Horizontal black bars
                field[(i * 48) + j] = 1
            elif ((i <= -j + 7) or (i >= j + 40) or (i >= -j + 87) or (i <= j - 40)): # Diagonal colored bars
                field[(i * 48) + j] = 1
            else:
                field[(i * 48) + j] = 0

    return field

def valueToString(value):
    if (value == 1):
        return "@"
    elif (value == 0):
        return "_"
    elif (value == 3):
        return "*"
    else:
        return "#"

def valid(x, y):
    if (x < 0 or x >= 48) or (y < 0 or y >= 48):
        return False
    
    if ((x >= 14 and x < 34) and ((y >= 0 and y < 10) or (y >= 38))): # Goals
        return False
    elif ((x >= 6 and x < 42) and (y >= 21 and y < 27)): # Center vertical bar
        return False
    elif ((x >= 5 and x < 11) or (x >= 37 and x < 43)) and ((y >= 13 and y < 35)): # Horizontal black bars
        return False
    elif ((x <= -y + 7) or (x >= y + 40) or (x >= -y + 87) or (x <= y - 40)): # Diagonal colored bars
        return False
    else:
        return True
    

    
def dist(init, next):
    return abs(math.sqrt(math.pow(init[0] - next[0], 2) + math.pow(init[1] - next[1], 2)))
    
def aStar(field, start, end):
    nodeId = 1
    startNode = Node(0, start[0], start[1], 0, dist(start, end), None)
    open = [startNode]
    closed = []
    exit = False
    path = []
    i = 0

    while len(open) > 0:
        q = heapq.heappop(open)
        #print("Searching Point" + str(i))
        i += 1
        x = q.getX()
        y = q.getY()
        field[(x * 48) + y] = 3
        printField(field)
        print("\n\n\n\n\n\n\n")
        #field[(x * 48) + y] = 0


        successors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y-1), (x+1, y+1), (x-1, y-1), (x-1, y+1)]

        for obj in successors:
            if (obj[0] == end[0] and obj[1] == end[1]):
                #print("Path Found")
                current = q
                while current is not None:
                    path.append(current)
                    current = current.getParent()

                exit = True
                break
            
            if valid(obj[0], obj[1]):
                g = q.getG() + dist((x, y), (obj[0], obj[1]))
                h = dist((obj[0], obj[1]), end)
                f = g + (0.5) * h
                #print("Searching Valid Successor")

                skip1 = True
                for node in open:
                    if node.getX() == obj[0] and node.getY() == obj[1]:
                        skip1 = False
                        break

                if (skip1):
                    skip2 = True
                    for node in closed:
                        if node.getX() == obj[0] and node.getY() == obj[1]:
                            skip2 = False
                            break

                    if (skip2):
                        open.append(Node(nodeId, obj[0], obj[1], g, h, q))
                        nodeId += 1

        closed.append(q)
        if (exit):
            break

    for node in path:
        field[(node.getX() * 48) + node.getY()] = 2

    return field, path

def printField(field):
    for i in range(48):
        string = ""
        for j in range(48):
            string += valueToString(field[(i * 48) + j])
            string += " "

        print(string)


field = setupField()

printField(field)

field, points = aStar(field, (0, 8), (47, 39))

printField(field)
