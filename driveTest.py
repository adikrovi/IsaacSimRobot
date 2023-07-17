from env import VexBotEnv
import math
import heapq
from Node import Node

from omni.isaac.kit import SimulationApp
import numpy as np

def m(inches):
    return inches / 39.37

simulation_app = SimulationApp({"headless": False, "anti_aliasing": 0})


################################################################
####################### World Setup ############################
################################################################
  
 
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd
from omni.isaac.dynamic_control import _dynamic_control
import omni.appwindow
import carb

my_world = World(stage_units_in_meters=1.0)
#self._my_world.scene.add_default_ground_plane()
field = add_reference_to_stage(usd_path="/home/opencav-krovi/VEX_AI/robot/FieldWithRobot.usd", prim_path="/World/Field")
vexbot_asset_path = "/home/opencav-krovi/VEX_AI/robot/vexbot.usd"
dc = _dynamic_control.acquire_dynamic_control_interface()
vexbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Body_Copy_1", 
        name="my_vexbot",
        wheel_dof_names=["dof_BL", "dof_BR", "dof_FL", "dof_FR"],
        create_robot=True,
        usd_path=vexbot_asset_path,
        position=np.array([m(24), m(-24), 0.05]),
        orientation=np.array([-250.0, 220.0, 250.0, 250.0]),
    )
)



################################################################
######################## Functions #############################
################################################################

def move_dist(distance, init):
    bot = dc.get_rigid_body("/World/Body_Copy_1/Body")
    print("Distance: " + str(distance))
    curr = dc.get_rigid_body_pose(bot).p
    print("Curr: " + str(curr))
    dist = math.sqrt(pow(curr[1]-init[1], 2) + pow(curr[0] - init[0], 2))
    print("Dist: " + str(dist - distance))

    if dist - distance < 0.01:
        leftDrive = 10 * (distance - dist)
        rightDrive = 10 * (distance - dist)
        return leftDrive, rightDrive
    else:
        leftDrive = 0
        rightDrive = 0
        return leftDrive, rightDrive
    
def find_ang(pos, currPos):
    bot = dc.get_rigid_body("/World/Body_Copy_1/Body")

    dX = pos[0] - currPos[0]
    dY = pos[1] - currPos[1]
    desAng = 0

    desAng = math.atan2(dY, dX)

    return desAng
    

def move_to_point(pos):
    global turnPidIntegral, turnPidDerivative, turnPidDrive, turnPidLastError, latPidIntegral, latPidDerivative, latPidDrive, latPidLastError
    bot = dc.get_rigid_body("/World/Body_Copy_1/Body")
    currAng   = math.atan2(2.0 * (dc.get_rigid_body_pose(bot).r[3] * dc.get_rigid_body_pose(bot).r[0] + dc.get_rigid_body_pose(bot).r[1] * dc.get_rigid_body_pose(bot).r[2]) , - 1.0 + 2.0 * (dc.get_rigid_body_pose(bot).r[0] * dc.get_rigid_body_pose(bot).r[0] + dc.get_rigid_body_pose(bot).r[1] * dc.get_rigid_body_pose(bot).r[1]))
    if (currAng <= 0):
        currAng += math.pi
    else:
        currAng -= math.pi

    currAng *= -1
    
    currX = dc.get_rigid_body_pose(bot).p[0] + (.075 * math.cos(currAng))
    currY = dc.get_rigid_body_pose(bot).p[1] + (.075 * math.sin(currAng))
    distance = math.sqrt(pow(pos[0] - currX, 2) + pow(pos[1] - currY, 2))
    desAng = find_ang((pos[0], pos[1]), (currX, currY))
    dAng = desAng - currAng


    turnKp = 4.3
    turnKi = 0.02
    turnKd = 0.0

    latKp = 5
    latKi = 0.01
    latKd = 0.0

    if (abs(dAng) > 0.05):
        
        if (turnKi != 0):
            if (abs(dAng) < turnintegralLimit):
                turnPidIntegral = turnPidIntegral + dAng
            else:
                turnPidIntegral = 0

        turnPidDerivative = dAng - turnPidLastError
        turnPidLastError = dAng

        turnPidDrive = (turnKp * dAng) + (turnKi * turnPidIntegral) + (turnKd * turnPidDerivative)
    else:
        turnPidIntegral = 0
        turnPidLastError = 0
        turnPidDerivative = 0
        turnPidDrive = 0

    if (abs(dAng) > math.pi):
        turnPidLastError = (2 * math.pi) - dAng

        turnPidDrive = -1 * ((turnKp * ((2 * math.pi) - dAng)))
    
    if (distance > 0.1):

        if (latKi != 0):
            if (abs(distance) < latintegralLimit):
                latPidIntegral = latPidIntegral + distance
            else:
                latPidIntegral = 0

        latPidDerivative = distance - latPidLastError
        latPidLastError = distance

        latPidDrive = (latKp * distance) + (latKi * latPidIntegral) + (latKd * latPidDerivative)

    else:
        latPidIntegral = 0
        latPidLastError = 0
        latPidDerivative = 0
        latPidDrive = 0


    leftPower = turnPidDrive + latPidDrive
    rightPower = -turnPidDrive + latPidDrive

    return currAng, desAng, leftPower, rightPower, currX, currY, distance


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
        #field[(x * 48) + y] = 0


        successors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y-1), (x+1, y+1), (x-1, y-1), (x-1, y+1)]

        for obj in successors:
            if (obj[0] == end[0] and obj[1] == end[1]):
                #print("Path Found")
                current = q
                while current is not None:
                    path = [current] + path
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

def fieldToSim(x, y):
    x = x * 3 # translate to inches
    y = y * 3

    x -= 72 # Center it
    y -= 72

    temp = x # rotate field 90 counter clockways
    x = -y
    y = temp

    x = m(x) # translate to meters
    y = m(y)

    return (x, y)


################################################################
###################### Start of Code ###########################
################################################################




turnPidIntegral = 0
turnintegralLimit = 0.1
latintegralLimit = 0.01
turnPidDerivative = 0
turnPidLastError = 0
turnPidDrive = 0
latPidIntegral = 0
latPidDerivative = 0
latPidLastError = 0
latPidDrive = 0
field = setupField()
field, pointsList = aStar(field, (16, 16), (40, 40))
counter = 0

points = []

for node in pointsList:
    points.append(fieldToSim(node.getX(), node.getY()))

for i in range(48):
    string = ""
    for j in range(48):
        string += valueToString(field[(i * 48) + j])
        string += " "

    print(string)

for p in points:
    print(p)

#Spawn Waypoints:
cubes = [None] * len(points)
k = 0
for p in points:
    cubes[k] = my_world.scene.add(
        VisualCuboid(
            prim_path="/new_cube_" + str(k),
            name="visual_cube_" + str(k),
            position=np.array([p[0], p[1], 0.05]),
            size=0.05,
            color=np.array([1.0, 0, 0]),
        )
    )
    k += 1



while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_playing():

        currAng, desAng, leftDrive, rightDrive, x, y, dista = move_to_point((points[counter][0], points[counter][1]))
        print("DesAng: " + str(desAng))
        print("CurrAng: " + str(currAng))
        print("X: " + str(x))
        print("Y: " + str(y))
        print("Dest: " + str(points[counter][0]) + ", " + str(points[counter][1]))
        print("\n")

        if (dista < m(15)):
            if counter < len(points) - 1:
                cubes[counter].color = np.array([0, 1.0, 0])
                counter += 1

        

        
        art = dc.get_articulation("/World/Body_Copy_1")
        dc.wake_up_articulation(art)
        joint_vels = [-rightDrive, -rightDrive, -leftDrive, -leftDrive, 0, 0]
        dc.set_articulation_dof_velocity_targets(art, joint_vels)
    else:
        turnPidIntegral = 0
        turnPidDerivative = 0
        turnPidLastError = 0
        turnPidDrive = 0

        latPidIntegral = 0
        latPidDerivative = 0
        latPidLastError = 0
        latPidDrive = 0


simulation_app.close()






 
   
 

