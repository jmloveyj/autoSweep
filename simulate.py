#!/usr/local/bin/python
#-*-coding:utf-8-*-   

from  Tkinter import *
import networkx as nx

DIRECTION_DELTA = {"E":(1,0),"W":(-1,0),"N":(0,-1),"S":(0,1)}
ROBOT_SIZE = 5
GAP = 2
COLOR_COVERED = "#FFDA89"
COLOR_CLEANED = "#FFFFFF"
COLOR_BOUNDARY = "#FEFFFF"
COLOR_DOOR = "#FDFFFF"

def calculateDistance(start,end):
    return abs(start[0] - end[0])+abs(start[1] - end[1])


def getConverseDirection(direction):
    if(direction == "E"):
        return "W"
    if(direction == "W"):
        return "E"
    if(direction == "N"):
        return "S"
    if(direction == "S"):
        return "N"


def get1StepCleanedRegion(robotPosition,robotDirection, pixelNum = 1):
    if(robotDirection == "W"):
        edgeStartX = robotPosition[0] - int(ROBOT_SIZE/2) - pixelNum
        edgeEndX = robotPosition[0] - int(ROBOT_SIZE/2) -1
        edgeStartY = robotPosition[1] - int(ROBOT_SIZE/2)
        edgeEndY = robotPosition[1] + int(ROBOT_SIZE/2)
    if(robotDirection == "E"):
        edgeStartX = robotPosition[0] + int(ROBOT_SIZE/2) +1
        edgeEndX = robotPosition[0] + int(ROBOT_SIZE/2) +pixelNum
        edgeStartY = robotPosition[1] - int(ROBOT_SIZE/2)
        edgeEndY = robotPosition[1] + int(ROBOT_SIZE/2)
    if(robotDirection == "S"):
        edgeStartX = robotPosition[0] - int(ROBOT_SIZE/2)
        edgeEndX = robotPosition[0] + int(ROBOT_SIZE/2)
        edgeStartY = robotPosition[1] + int(ROBOT_SIZE/2) +1
        edgeEndY = robotPosition[1] +int(ROBOT_SIZE/2) +pixelNum
    if(robotDirection == "N"):
        edgeStartX = robotPosition[0] - int(ROBOT_SIZE/2)
        edgeEndX = robotPosition[0] + int(ROBOT_SIZE/2)
        edgeStartY = robotPosition[1] - int(ROBOT_SIZE/2) -pixelNum
        edgeEndY = robotPosition[1] - int(ROBOT_SIZE/2) -1   
    return (edgeStartX,edgeEndX,edgeStartY,edgeEndY)
            
         

class Map(nx.Graph,Canvas):
    
    def __init__(self,parent, iniBackground):
        nx.Graph.__init__(self)
        Canvas.__init__(self,parent,bd=3,relief=RIDGE,width=500, height=300)
        self.im = PhotoImage(file= iniBackground)
        self.create_image(ROBOT_SIZE,ROBOT_SIZE,anchor=NW, image= self.im)
        self.pack(side=RIGHT)
        self.uncoveredReg =[]
        self.uncoveredRoom = []
        self.currentRoom = None
        self.coveredReg =[]
        self.currentReg = None
        self.currentX = int(ROBOT_SIZE/2) +GAP  # robot center coordinate because robot size is 5*5
        self.currentY = int(ROBOT_SIZE/2) +GAP
        self.index=0
        self.NODE_TYPE_SHAPE_MAP = \
            {'corner':self.__drawCornerNode, 'obstacle':self.__drawObstacleNode,'joint':self.__drawJointNode}
        self.EDGE_TYPE_SHAPE_MAP = \
            {'free':self.__drawFreeEdge, 'upObtacle':self.__drawUpObtacleEdge,'downObtacle':self.__drawDownObtacleEdge}

    def moveUpdate(self,robotDirection):
        (edgeStartX,edgeEndX,edgeStartY,edgeEndY) = get1StepCleanedRegion((self.currentX,self.currentY),robotDirection)
        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                if(self.im.get(i,j)=="220 220 220"):
                    self.im.put(COLOR_CLEANED,(i,j))
        
        self.currentX = self.currentX + DIRECTION_DELTA[robotDirection][0];
        self.currentY = self.currentY + DIRECTION_DELTA[robotDirection][1]

    def ifRoomFinished(self):
        return not self.uncoveredReg

    def ifFinished(self):
        return not self.uncoveredRoom


    def setCurrentPos(self,x,y):
        self.currentX = x
        self.currentY = y
        edgeStartX = x - int(ROBOT_SIZE/2)
        edgeEndX = x + int(ROBOT_SIZE/2)
        edgeStartY = y - int(ROBOT_SIZE/2)
        edgeEndY = y + int(ROBOT_SIZE/2)

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                self.im.put(COLOR_CLEANED,(i,j))

    def ifBump(self,robotDirection):
        robotPosition = (self.currentX,self.currentY)
        (edgeStartX, edgeEndX, edgeStartY, edgeEndY) = get1StepCleanedRegion(robotPosition,robotDirection)

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                if(self.im.get(i,j)=="253 255 255"):
                    return True
        return False



    def buildVirtalWall(self,start, end,currentDirection):
        if start[1] > end[1]:
            upLimit = start[1]
            downLimit = end[1]
        else:
            upLimit = end[1]
            downLimit = start[1]

        for i in range(start[0],end[0]+1):
            for j in range(downLimit,upLimit+1):
                self.im.put(COLOR_DOOR,(i,j))
       

        self.currentRoom = {'pos':start, 'direction':getConverseDirection(currentDirection)}
        self.uncoveredRoom.append({'pos':start, 'direction':currentDirection})

    def ifCloseToVirtualWall(self,direction):
        currentPos = (self.currentX, self.currentY)
        (edgeStartX, edgeEndX, edgeStartY, edgeEndY) = get1StepCleanedRegion(currentPos, direction, pixelNum = ROBOT_SIZE)
        
       
        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
     
                if(self.im.get(i,j)=="253 255 255"):
                    return True
        return False



    def setCurrentRoom(self,room):
        self.currentRoom = room
        self.uncoveredRoom.remove(room)

    def setCurrentRegion(self,region):
        self.currentReg = region
        self.__delUncoveredReg(region)


    def getRouteToNextRoom(self):
        currentPos = (self.currentX, self.currentY)

        shortestDistance = 999
        nextRoom = None
      
        for item in self.uncoveredRoom:
        
            distance= calculateDistance(currentPos, item['pos'])
            
            if (distance < shortestDistance):
                shortestDistance = distance
                nextRoom = item
            if shortestDistance==0:
                break
    
        path = self.__getPath(currentPos, nextRoom['pos'])
       
        return {'path':path, 'targetRoom':nextRoom, 'targetRegion':{}}


    # route = {path: [position, position], sweepDirection:"N"}
    def getRouteToNextRegionBoundary(self):
        currentPos = (self.currentX, self.currentY)
        
        shortestDistance = 999
        nextRegPoint = None
        nextReg = None
  
        for item in self.uncoveredReg:
            if item['up']:
                distance, point= self.__getDistanceFromPoint2Edge(currentPos, item['up'])
            else:
                distance,point = self.__getDistanceFromPoint2Edge(currentPos, item['down'])
            if (distance < shortestDistance):
                shortestDistance = distance
                nextRegPoint = point
                nextReg = item
            if shortestDistance==0:
                break
    
        path = self.__getPath(currentPos, nextRegPoint)
       
        return {'path':path, 'targetRegion':nextReg,'targetRoom':{}}

        


    def addNode(self,position, type):
        self.add_node(position,{'type':type})
        self.NODE_TYPE_SHAPE_MAP[type](position)
        return position

    def addEdge(self,start, end,type):
        self.add_edge(start,end,{'type':type})
        self[start][end]['weight'] = calculateDistance(start,end)
        self.EDGE_TYPE_SHAPE_MAP[type](start,end)

    def updateRegion(self,nodeList):
        upBoundary,downBoundary = self.__segmentRegionByBoundary(nodeList)

        self.__updateUncoveredReg(upBoundary, downBoundary)

        if not self.currentReg:
            self.currentReg = self.uncoveredReg.pop()

            return

        self.combineCurrentBoundary()
        

    def __getDistanceFromPoint2Edge(self,point, edge):
        distance1 = calculateDistance(point, edge[0])
        distance2 = calculateDistance(point, edge[1])

        if distance1<distance2:
            return distance1, edge[0] 
        else:
            return distance2, edge[1] 

    # a* 简洁算法
    def __getPath(self, start, end):
        path = []
        current = start

        path.append(start)

         
        while (current != end):
            neighbourList = self.__getNeighbour(current)
            F = 999
            nextPoint = None
            for neighbourPoint in neighbourList:
                if self.__ifBlock(neighbourPoint):
                    continue
                if neighbourPoint in path:
                    continue
                 

                weight = self.__calculateF(neighbourPoint, start, end)

                if( weight< F):
                    F = weight
                    nextPoint = neighbourPoint
            current = nextPoint
       
            path.append(current)
 
        return path

    def ifTouchBoundary(self,direction):
        robotPosition = (self.currentX, self.currentY)
        (edgeStartX, edgeEndX, edgeStartY, edgeEndY) = get1StepCleanedRegion(robotPosition,direction)
        

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                if(self.im.get(i,j)=="254 255 255"):
                    return True                    
        return False 

    def __ifBlock(self,point):
        edgeStartX = point[0] - int(ROBOT_SIZE/2)
        edgeEndX = point[0] + int(ROBOT_SIZE/2)
        edgeStartY = point[1] - int(ROBOT_SIZE/2)
        edgeEndY = point[1] + int(ROBOT_SIZE/2)

        # if ((edgeStartX <GAP) or (edgeEndX> Env.WIDTH-GAP) or (edgeStartY <GAP) or (edgeEndY>Env.HEIGHT-GAP)):
        #     return True

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                if(self.im.get(i,j)=="220 220 220"):
                    return True                    
        return False 

    def __calculateF(self, point, start, end):
        G = calculateDistance(point, start)
        H = calculateDistance(point, end)
        return H



    def __getNeighbour(self,current):
        neighbourList =[]
        neighbourList.append((current[0]+1, current[1]))
        neighbourList.append((current[0], current[1]-1))
        neighbourList.append((current[0]-1, current[1]))
        neighbourList.append((current[0], current[1]+1))
        
        
       
        return neighbourList


    def __updateUncoveredReg(self,upBoundary,downBoundary):
        for item in upBoundary:
            self.__addUncoveredReg({'up':item, 'down':()})

        for item in downBoundary:
            self.__addUncoveredReg({'up':(),'down':item})

    def __addUncoveredReg(self, region):
        
        self.__buildVirtualBoundary(region)
        self.uncoveredReg.append(region)

    def __delUncoveredReg(self, region):
        self.__buildVirtualBoundary(region,color=COLOR_CLEANED)
        self.uncoveredReg.remove(region)

    def __buildVirtualBoundary(self,region,color=COLOR_BOUNDARY):
        edge = None
        if(region['up']):
            edge = region['up']
            edge = ((edge[0][0],edge[0][1]+2), (edge[1][0],edge[1][1]+2))
        else:
            edge = region['down']
            edge = ((edge[0][0],edge[0][1]-2), (edge[1][0],edge[1][1]-2))


        for i in range(edge[0][0], edge[1][0]+1):
            for j in range(edge[0][1], edge[1][1]+1):
                self.im.put(color,(i,j))


    def combineCurrentBoundary(self):
        if not self.currentReg:
            return
        if not self.currentReg['up']:
            region = self.__findClosedRegion(self.currentReg['down'], 'up')
            self.currentReg['up'] = region['up']
            self.__delUncoveredReg(region)
        elif not self.currentReg['down']:
            region = self.__findClosedRegion(self.currentReg['up'], 'down')
            
            
            self.currentReg['down'] = region['down']
            self.__delUncoveredReg(region)
    
        self.__drawFreeEdge(self.currentReg['up'][0], self.currentReg['down'][0])
        self.__drawFreeEdge(self.currentReg['up'][1], self.currentReg['down'][1])
        self.coveredReg.append(self.currentReg)


    def __findClosedRegion(self,edge, direction):
        anotherRegion = ()
        for item in self.uncoveredReg:
            if not item[direction]:
                continue

            if (direction == 'up') and (item[direction][0][1] >= edge[0][1]):
                continue
            if (direction == 'down') and (item[direction][0][1] <= edge[0][1]):
                continue
            if not anotherRegion:
                anotherRegion = item
                continue

            if calculateDistance(edge[0],item[direction][0])< calculateDistance(edge[0],anotherRegion[direction][0]):
                anotherRegion = item

        return anotherRegion
        

    def __drawCornerNode(self,position):
        pos = self.__convertPos(position)
        self.create_rectangle(pos[0]-1.5,pos[1]-1.5,pos[0]+1.5,pos[1]+1.5,\
            fill="red",outline="red") 

    def __drawObstacleNode(self,position):
        pos = self.__convertPos(position)
        self.create_oval(pos[0]-1.5,pos[1]-1.5,pos[0]+1.5,pos[1]+1.5,\
            outline="blue")
    def __drawJointNode(self,position):
        pos = self.__convertPos(position)
        self.create_oval(pos[0]-1.5,pos[1]-1.5,pos[0]+1.5,pos[1]+1.5,\
            fill="black")
    def __drawFreeEdge(self,start,end):
        newStart = self.__convertPos(start)
        newEnd = self.__convertPos(end)
        self.create_line(newStart[0], newStart[1],newEnd[0],newEnd[1])
    
    def __drawUpObtacleEdge(self,start,end):
        newStart = self.__convertPos(start)
        newEnd = self.__convertPos(end)
        self.create_line(newStart[0], newStart[1],newEnd[0],newEnd[1],dash=True, fill="red")

    def __drawDownObtacleEdge(self,start,end):
        newStart = self.__convertPos(start)
        newEnd = self.__convertPos(end)
        self.create_line(newStart[0], newStart[1],newEnd[0],newEnd[1],dash=True, fill="blue")

    def __convertPos(self,position):
        return (position[0]+5, position[1]+5)


    def __segmentRegionByBoundary(self,nodeList):
        upRegionBoundary = []
        downRegionBoundary = []

        up = ()
        down=()

        length = len(nodeList)
        for i in range(length-1):
            edge = self.get_edge_data(nodeList[i], nodeList[i+1])
            if(edge['type'] == 'free') or (edge['type']=="upObtacle"):
                if not up:
                    up = (nodeList[i], nodeList[i+1])
                else: 
                    if (up[1] == nodeList[i]):
                        up = (up[0], nodeList[i+1])
                    else:
                        upRegionBoundary.append(up)
                        up = (nodeList[i], nodeList[i+1])
            if(edge['type'] == 'free') or (edge['type']=="downObtacle"):
                if not down:
                    down = (nodeList[i], nodeList[i+1])
                else: 
                    if (down[1] == nodeList[i]):
                        down = (down[0], nodeList[i+1])
                    else:
                        downRegionBoundary.append(down)
                        down = (nodeList[i], nodeList[i+1])

        if up:
            upRegionBoundary.append(up)

        if down:
            downRegionBoundary.append(down)

        return upRegionBoundary,downRegionBoundary





#控制行走策略， 通过在走每一步前， 安放检查点， 改变机器人的 行走方向，达到控制的目的
class MoveCtl():
    def __init__(self, robot):
        self.robot = robot
    
    # return 是否真正移动
    def decideNextMove(self):
        return True


    def endControl(self):
        self.robot.checkProgress()

    
class FindCornerCtl(MoveCtl):
    def __init__(self,robot,stepDirection):
        MoveCtl.__init__(self, robot)
        self.robot.currentDirection = stepDirection
        self.stepNum =0

    def decideNextMove(self):
        if(self.stepNum < ROBOT_SIZE):
            self.stepNum = self.stepNum +1
            return True
        else:
            self.robot.currentDirection = 'N'

        ifBump = self.robot.ifBump()

        if(ifBump):
            self.robot.moveCtl = ZigzagMoveCtl(self.robot,'S')
            return False

        return True


class ZigzagMoveCtl(MoveCtl):
    global ROBOT_SIZE
    global GAP
    def __init__(self, robot, sweepDirection, ifAlterSliceAtFirst = False, firstSliceDirection="W"):
        MoveCtl.__init__(self, robot)
        self.alterDistance = 0
        self.alterMaxDistance = ROBOT_SIZE
        self.sweepDirection = sweepDirection
        self.lastSliceDirection = firstSliceDirection
        self.currentSlice = {}
        self.ifSliceDeadHead = False
        self.ifLastSlice = False
        self.robot.stateText = "弓形清扫"

        if (ifAlterSliceAtFirst):
            self.robot.currentDirection = sweepDirection
            self.__act = self.__processAlterSlice
        else:
            self.__act = self.__processSlice
            self.robot.currentDirection = firstSliceDirection
            self.ifSliceDeadHead = False #这个slice是否空驶


    def __ifInCornerAtStart(self):
        range = self.robot.getRangeData()
        if range['E']>range['W']:

        if(range[getConverseDirection(self.robot.currentDirection)]> GAP):
            return False
        else:
            return True

    def decideNextMove(self):

        return self.__act()


    def __processSlice(self): 
        ifBump = self.robot.ifBump()
        if(ifBump):
            if(self.ifLastSlice):
                self.robot.map.combineCurrentBoundary()
                assert(self.robot.ifCurrentRegionFinished())
                self.endControl()
                return False
            if(self.ifSliceDeadHead):
            
                self.robot.currentDirection = getConverseDirection(self.robot.currentDirection) 
                self.lastSliceDirection = self.robot.currentDirection
                self.__act = self.__processSlice

                self.__handleSliceBegin()
                self.ifSliceDeadHead = False
       
                
            else:
                self.__handleSliceEnd()
                self.__act = self.__processAlterSlice
                range = self.__getRangeData()
                if(range['head'] >(ROBOT_SIZE*3)):
                    self.alterMaxDistance = 3
                else:
                    self.alterMaxDistance = ROBOT_SIZE
                self.alterDistance = 0
                self.lastSliceDirection = self.robot.currentDirection
                self.robot.currentDirection = self.sweepDirection
            return False
        if(self.__ifLeftRightDoorFound()):
            
            self.__buildVirtalWall()
            self.__handleSliceEnd()
            self.alterMaxDistance = ROBOT_SIZE
            self.lastSliceDirection = self.robot.currentDirection
            self.robot.currentDirection = getConverseDirection(self.lastSliceDirection)
            self.__act = self.__processAlterSlice
            return False

        if(not self.ifSliceDeadHead):
            self.__handleSliceInProgress()
        return True

    def __buildVirtalWall(self):
        point = self.currentSlice['doorPoint'][-1]['pos']
        start = point
        if(self.sweepDirection == "S"):
            end = (start[0], start[1] + ROBOT_SIZE*10) 
        else:
            end = (start[0], start[1] - ROBOT_SIZE*10)
        self.robot.map.buildVirtalWall(start,end,self.robot.currentDirection)

    def __ifLeftRightDoorFound(self):
        if (not self.currentSlice):
            return False
        if (not self.currentSlice['doorPoint']):
            return False

        if len(self.currentSlice['doorPoint'])<2:
            return False

        if(calculateDistance(self.currentSlice['doorPoint'][0]['pos'],self.currentSlice['start'])<ROBOT_SIZE):
            self.currentSlice['doorPoint'] =[]
            return False

        corridorWidth = abs(self.currentSlice['doorPoint'][1]['pos'][0] - self.currentSlice['doorPoint'][0]['pos'][0])

        if (corridorWidth<ROBOT_SIZE*3) and (corridorWidth>ROBOT_SIZE):
            return True

        self.currentSlice['doorPoint'] =[]
        return False



    def __processAlterSlice(self):
        if(self.robot.ifTouchBoundary()):
            self.ifLastSlice = True
            self.robot.currentDirection = getConverseDirection(self.lastSliceDirection)
            self.__act = self.__processSlice
            self.ifSliceDeadHead = True 

            return False

        ifBump = self.robot.ifBump()
        if(ifBump):
            if(self.alterDistance>0):
                self.robot.currentDirection = getConverseDirection(self.lastSliceDirection)
                self.__act = self.__processSlice

                self.__handleSliceBegin()
            else:
                self.robot.currentDirection = getConverseDirection(self.lastSliceDirection)
                self.backDistance = 0
                self.__act = self.__processBackOneRobot
            return False

        if(self.alterDistance == self.alterMaxDistance):
            self.robot.currentDirection = getConverseDirection(self.lastSliceDirection)
            self.__act = self.__processSlice
            if(self.__ifBackEmpty()):
                self.ifSliceDeadHead = True
       
            else:
      
                self.__handleSliceBegin()
            return False
        self.alterDistance +=1
        return True

    def __ifBackEmpty(self):
        

        range = self.robot.getRangeData()
        backSpace = range[self.lastSliceDirection]
        if (backSpace > ROBOT_SIZE*3):
            if(self.robot.map.ifCloseToVirtualWall(self.lastSliceDirection)):
                return False
            return True

        return False




    

    def __processBackOneRobot(self):
        ifBump = self.robot.ifBump()
        if ifBump:
            print "very very interesting"
            self.robot.currentDirection = self.sweepDirection
            self.__act = self.__processAlterSlice
            return False

        if(self.backDistance == ROBOT_SIZE):
            self.robot.currentDirection = self.sweepDirection
            self.__act = self.__processAlterSlice
            return False
        self.backDistance +=1
        return True

    def __handleSliceInProgress(self):
        if(not self.currentSlice):
            return None
        self.__checkCriticality()

        


    def __handleSliceBegin(self):

        self.currentSlice["start"] = self.robot.getCurrentPos()
        self.currentSlice["end"] = (999,999)
        self.currentSlice['direction'] = self.robot.currentDirection

        self.currentSlice['criticality'] = []
        self.currentSlice['doorPoint'] = []

        self.__checkCriticality()

    def __handleSliceEnd(self):         
        self.currentSlice['end'] = self.robot.getCurrentPos()

        self.__checkCriticality()

        self.__filterEnd()

        #普通slice， 没有临界点
        if(len(self.currentSlice['criticality']) ==2) and \
        (self.currentSlice['criticality'][0]['rangeStatus'] == (False,False)):
            return

        
        self.robot.buildBoundary(self.currentSlice['criticality'])
        if(self.robot.ifCurrentRegionFinished()):
            self.endControl()



    #避免球形，斜边，凹形slice边界造成短距离的障碍物误判
    def __filterEnd(self):
        if (len(self.currentSlice['criticality']) > 2):
            startNode = self.currentSlice['criticality'][0]
            secondNode = self.currentSlice['criticality'][1]
            startDistance = calculateDistance(startNode['pos'], secondNode['pos'])
            if(startNode['rangeStatus'] != (False,False) and secondNode['rangeStatus'] != (False,False)):
                self.currentSlice['criticality'][0]['rangeStatus'] = (False,False)
            if (startDistance < (ROBOT_SIZE)):
                startNode['rangeStatus'] = secondNode['rangeStatus']
                self.currentSlice['criticality'].remove(secondNode)

        if (len(self.currentSlice['criticality']) > 2):
            endNode = self.currentSlice['criticality'][-1]
            secondEndNode = self.currentSlice['criticality'][-2]
            thirdEndNode = self.currentSlice['criticality'][-3]
            endDistance = calculateDistance(endNode['pos'], secondEndNode['pos'])
            if(thirdEndNode['rangeStatus'] != (False,False) and secondEndNode['rangeStatus'] != (False,False)):
                self.currentSlice['criticality'][-1]['rangeStatus'] = (False,False)
                self.currentSlice['criticality'][-2]['rangeStatus'] = (False,False)
            if (endDistance < (ROBOT_SIZE)):
                endNode['rangeStatus'] = secondEndNode['rangeStatus']
                self.currentSlice['criticality'].remove(secondEndNode)


        if (len(self.currentSlice['criticality']) == 2):
            if(self.currentSlice['criticality'][0]['rangeStatus'] != self.currentSlice['criticality'][1]['rangeStatus']):
                self.currentSlice['criticality'][0]['rangeStatus'] = (False,False)
                self.currentSlice['criticality'][1]['rangeStatus'] = (False,False)

        
    def __getLastPos(self,currentPos):
        currentDirection = self.robot.currentDirection
        if(currentDirection == "E"):
            return(currentPos[0]-1, currentPos[1])
        if(currentDirection == "W"):
            return(currentPos[0]+1, currentPos[1])
        if(currentDirection == "N"):
            return(currentPos[0], currentPos[1]+1)
        if(currentDirection == "S"):
            return(currentPos[0], currentPos[1]-1)
   
    def __checkCriticality(self):
        currentPosition = self.robot.getCurrentPos()

        range = self.__getRangeData()
        
        
        ifUpClose = (range['up'] < ROBOT_SIZE+2)
        ifBelowClose = (range['down'] < ROBOT_SIZE+2)

        if(ifUpClose and ifBelowClose):
            if(range['up']>range['down']):
                ifUpClose = False 
            else:
                ifBelowClose = False
        # ifForwardClose = (range['head'] < ROBOT_SIZE+GAP+1)
        upDownSpace = range['up'] + range['down']
        ifUpDownClose = (upDownSpace<ROBOT_SIZE*10)

        if(ifUpDownClose) and (not self.currentSlice['doorPoint']):
            self.currentSlice['doorPoint'].append({'pos':currentPosition, 'upDownCloseStatus':ifUpDownClose})

        if (self.currentSlice['doorPoint']) and (ifUpDownClose != self.currentSlice['doorPoint'][-1]['upDownCloseStatus']):
            self.currentSlice['doorPoint'].append({'pos':currentPosition, 'upDownCloseStatus':ifUpDownClose})


        if(self.currentSlice["start"] == currentPosition) and (not self.currentSlice['criticality']):
            self.currentSlice['criticality'].append({'pos':currentPosition, 'rangeStatus':(ifUpClose,ifBelowClose)})
            return 
    

        if(self.currentSlice['end'] == currentPosition):
            self.currentSlice['criticality'].append({'pos':currentPosition, 'rangeStatus':(ifUpClose,ifBelowClose)})
            return 
  

        if (self.currentSlice['criticality'][-1]['rangeStatus'] != (ifUpClose,ifBelowClose)):
            self.currentSlice['criticality'].append({'pos':currentPosition, \
                'rangeStatus':(ifUpClose,ifBelowClose)})
          

        return 


    # return range of slice direction, sweep direction and anti-sweep direction
    def __getRangeData(self):
        range = self.robot.getRangeData()
     
        head = range[self.robot.currentDirection]
        up = range['N']
        down = range['S']

        return {'head':head,'up':up, 'down':down}

class NavMoveCtl(MoveCtl):
    def __init__(self, robot, route):
        MoveCtl.__init__(self, robot)
        self.path = route['path']
        self.targetRegion = route['targetRegion']
        self.targetRoom = route['targetRoom']
        self.pathIndex = 1

    def decideNextMove(self):
        if (self.pathIndex == len(self.path)):
            if(self.targetRegion):
                self.__startSweep()
            elif(self.targetRoom):
                self.__findCorner()
            return False

        currentPos = self.robot.getCurrentPos()
        self.robot.currentDirection = self.__getMoveDirection(currentPos, self.path[self.pathIndex])

        self.pathIndex+=1
        return True

    def __findCorner(self):
        self.robot.setCurrentRoom(self.targetRoom)
        self.robot.moveCtl = FindCornerCtl(self.robot, self.targetRoom['direction'])

    def __startSweep(self):
        self.robot.setCurrentRegion(self.targetRegion)

        sweepDirection = "S"
        targetEdge =None
        firstSliceDirection = None
        if self.targetRegion['down']:
            sweepDirection ='N'
            targetEdge = self.targetRegion['down']
        else:
            sweepDirection = 'S'
            targetEdge = self.targetRegion['up']
        currentPos = self.robot.getCurrentPos()
 
        if currentPos == targetEdge[0]:
            firstSliceDirection = "W"
        else:
            firstSliceDirection = "E"
        
        
        self.robot.moveCtl = ZigzagMoveCtl(self.robot,sweepDirection, \
            ifAlterSliceAtFirst = True, firstSliceDirection=firstSliceDirection)


    def __getMoveDirection(self,start,end):
        if start[0] < end[0]:
            return 'E'

        if start[0] > end[0]:
            return 'W'
        if start[1] < end[1]:
            return 'S'

        if start[1] > end[1]:
            return 'N'




class Robot():
    global DIRECTION_DELTA
    global ROBOT_SIZE
    global GAP
    def __init__(self, env,map,statistic):
        self.env = env
        self.map = map
        self.statistic = statistic
        self.velocity = 0
        self.state = "move"
        self.map.setCurrentPos(int(ROBOT_SIZE/2) +GAP,int(ROBOT_SIZE/2) +GAP )
        self.env.clearInitRobot(self.getCurrentPos())
        self.appearance= env.create_rectangle(7,7,12,12, fill="red") 
        self.currentDirection = "E"
        self.stateText = "弓形清扫"
        self.moveCtl = ZigzagMoveCtl(self,"S")
        self.move()

    def checkProgress(self):
        if not self.map.ifRoomFinished():
            self.__navToNextRegion()
            return

        if not self.map.ifFinished():
            self.__navToNextRoom()
            return 

        self.stateText = "清扫结束"
        self.stopMove()
        return
        

    def __navToNextRegion(self):
        # route = {'path':path, 'targetRegion':edge}
        route = self.map.getRouteToNextRegionBoundary()
        self.stateText = "导航最近未清扫区域"
        self.moveCtl = NavMoveCtl(self, route)

    def __navToNextRoom(self):
        # route = {'path':path, 'targetRegion':edge}
        route = self.map.getRouteToNextRoom()
        self.stateText = "导航最近房间"
        self.moveCtl = NavMoveCtl(self, route)


    def setCurrentRegion(self,region):
        self.map.setCurrentRegion(region)

    def setCurrentRoom(self,room):
        self.map.setCurrentRoom(room)


    def stopMove(self):
        self.state  = "stop"

    def resumeMove(self):
        self.state  = "move"

    def buildBoundary(self, criticality):
        length = len(criticality)
        nodeList =[]

        for i in range(length):
            if(i==0 or i==(length-1)):
                if criticality[i]['rangeStatus'] == (False,False):
                    nodeList.append(self.map.addNode(criticality[i]['pos'],'joint'))
                else:
                    nodeList.append(self.map.addNode(criticality[i]['pos'],'corner'))
            else:
                nodeList.append(self.map.addNode(criticality[i]['pos'],'obstacle'))
    
        for i in range(length-1):
            if criticality[i]['rangeStatus'] == (False,False):
                self.map.addEdge(nodeList[i],nodeList[i+1],"free")
            elif criticality[i]['rangeStatus'] == (True,False):
                self.map.addEdge(nodeList[i],nodeList[i+1],"upObtacle")
            else:
                self.map.addEdge(nodeList[i],nodeList[i+1],"downObtacle")

        if(self.currentDirection == "W"):
            nodeList.reverse()

        self.map.updateRegion(nodeList)


    def ifCurrentRegionFinished(self):
        currentReg = self.map.currentReg
        return (currentReg['up'] and currentReg['down'])


    #速度分解
    def move(self):
        if(self.velocity == 0 ) or (self.state != "move"):
            self.env.after(1, self.move)
            return

        for i in range(self.velocity):
            if(self.moveCtl.decideNextMove()):
                self.__moveOnePixel()
                    

        self.env.after(1, self.move)


    def getCurrentPos(self):
        return (self.map.currentX, self.map.currentY)

    
    def getRangeData(self):
        return self.env.getRangeData(self.getCurrentPos())
        

    def ifBump(self,direction='default'):
        if (direction == 'default'):
            direction = self.currentDirection
        return self.env.ifBump(self.getCurrentPos(), direction) or self.map.ifBump(direction)

    def ifTouchBoundary(self):
        return self.map.ifTouchBoundary(self.currentDirection)



    def setVelocity(self, velocity):
        self.velocity = velocity

    def periodicUpdateStatistic(self):
        statistic = self.env.getStatistic()
        self.statistic.coverageRate.set(statistic['coverageRate'])
        self.statistic.repeatedRate.set(statistic['repeatedRate'])

        pos = self.getCurrentPos()
        self.statistic.robotPos.set("(%d,%d)" %(pos[0],pos[1]))

        self.statistic.robotState.set(self.stateText)

        range = self.getRangeData()
        if(self.currentDirection == "E"):
            self.statistic.head.set(range['E'])
            self.statistic.left.set(range['N'])
            self.statistic.right.set(range['S'])

        if(self.currentDirection == "W"):
            self.statistic.head.set(range['W'])
            self.statistic.left.set(range['S'])
            self.statistic.right.set(range['N'])

        if(self.currentDirection == "N"):
            self.statistic.head.set(range['N'])
            self.statistic.left.set(range['W'])
            self.statistic.right.set(range['E'])

        if(self.currentDirection == "S"):
            self.statistic.head.set(range['S'])
            self.statistic.left.set(range['E'])
            self.statistic.right.set(range['W'])


        self.env.after(100,self.periodicUpdateStatistic)

 

    def __moveOnePixel(self):
        self.env.move(self.appearance, self.getCurrentPos(), self.currentDirection)
        self.map.moveUpdate(self.currentDirection)



class Env(Canvas):
    global DIRECTION_DELTA
    global ROBOT_SIZE
    global GAP
    WIDTH = 500 
    HEIGHT = 296 

    def __init__(self, parent, iniBackground):
        Canvas.__init__(self,parent,bd=3,relief=RIDGE,width=Env.WIDTH, height=Env.HEIGHT)
        self.im = PhotoImage(file= iniBackground)
        self.create_image(ROBOT_SIZE,ROBOT_SIZE,anchor=NW, image= self.im)
        self.pack()
        self.totalArea = 0
        self.clearedArea = ROBOT_SIZE*ROBOT_SIZE
        self.repeatedArea = 0
        self.__calFreeArea()

    def clearInitRobot(self, robotPosition):
        edgeStartX = robotPosition[0] - int(ROBOT_SIZE/2)
        edgeEndX = robotPosition[0] + int(ROBOT_SIZE/2)
        edgeStartY = robotPosition[1] - int(ROBOT_SIZE/2)
        edgeEndY = robotPosition[1] + int(ROBOT_SIZE/2)

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                self.im.put(COLOR_COVERED,(i,j))
                self.clearedArea+=1


    def move(self,item,robotPosition, robotDirection):
        Canvas.move(self,item,DIRECTION_DELTA[robotDirection][0],DIRECTION_DELTA[robotDirection][1])
        
        (edgeStartX, edgeEndX, edgeStartY, edgeEndY) = get1StepCleanedRegion(robotPosition,robotDirection)

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                assert(self.im.get(i,j)!="0 0 0")
                if(self.im.get(i,j)=="255 255 255"):
                    self.im.put(COLOR_COVERED,(i,j))
                    self.clearedArea+=1
                else:
                    self.repeatedArea+=1


    def __calFreeArea(self):
        for i in range(Env.WIDTH):
            for j in range(Env.HEIGHT):
                r=self.im.get(i,j)
                if(r == "255 255 255"):
                    self.totalArea+=1
    

    # Range detected in (3 ~ 50). for convinience, the distance is calculated from center of robot
    def getRangeData(self, robotPosition):
        MAX_RANGE = 50
        MIN_RANGE = int(ROBOT_SIZE/2)
        east = 999
        west = 999
        north = 999
        south = 999
        
        for i in range(MIN_RANGE,MAX_RANGE +1):
            if((east==999)and ((robotPosition[0] + i) == Env.WIDTH-1)):
                east = i -1
                
            if((east==999) and (self.im.get(robotPosition[0]+i,robotPosition[1])=="0 0 0")):
                east = i -1

            if((west ==999) and ((robotPosition[0] - i) == 0)):
                west = i -1
                
            if((west ==999) and (self.im.get(robotPosition[0]-i,robotPosition[1])=="0 0 0")):
                west = i -1

            if((south==999) and ((robotPosition[1] + i) == Env.HEIGHT-1)):
                south = i -1
                
            if((south==999) and (self.im.get(robotPosition[0],robotPosition[1]+i)=="0 0 0")):
                south = i-1

            if((north==999) and ((robotPosition[1] - i) == 0)):
                north = i-1
                
            if((north==999) and (self.im.get(robotPosition[0],robotPosition[1]-i)=="0 0 0")):
                north = i-1

            if((east != 999) and (west != 999) and (south != 999) and (north != 999)):
                break;
                

        return {'W':west, 'E':east, 'N':north, 'S':south}


    def ifBump(self,robotPosition, robotDirection):
        (edgeStartX, edgeEndX, edgeStartY, edgeEndY) = get1StepCleanedRegion(robotPosition,robotDirection)

        if ((edgeStartX <1) or (edgeEndX> Env.WIDTH-2) or (edgeStartY <1) or (edgeEndY>Env.HEIGHT-2)):
            return True

        for i in range(edgeStartX, edgeEndX+1):
            for j in range(edgeStartY, edgeEndY+1):
                if(self.im.get(i,j)=="0 0 0"):
                    return True
        return False

    def getStatistic(self):
        coverageRate = "%.2f%%" %(self.clearedArea*100/self.totalArea) 
        repeatedRate = "%.2f%%" %(self.repeatedArea*100/self.clearedArea)
    
        return {'coverageRate':coverageRate, 'repeatedRate':repeatedRate}

class Statistic():
    def __init__(self):
        self.coverageRate = StringVar()
        self.repeatedRate = StringVar()
        self.robotPos = StringVar()
        self.head = StringVar()
        self.left = StringVar()
        self.right = StringVar()
        self.robotState = StringVar()

class App():
    def __init__(self, root):
        self.statistic = Statistic()
        self.master = Frame(root,padx=25)
        self.master.pack()
        self.showSweepView()
        self.showGraphView()
        self.showCommand()
        self.showDashBoard()
        self.robot=Robot(self.env,self.map, self.statistic)
        self.run()

    def __restart(self):
        self.master.destroy()
        self.master = Frame(root,padx=25)
        self.master.pack()
        self.showSweepView()
        self.showGraphView()
        self.showCommand()
        self.showDashBoard()
        self.robot=Robot(self.env,self.map, self.statistic)
        self.run()




    def showSweepView(self):
        sweepFrame = Frame(self.master,padx=25)
        sweepFrame.grid(row=0, column = 0)
        title = Label(sweepFrame,text="清扫视图",font=("Times", "16", "bold italic"))
        title.pack()
        self.env = Env(sweepFrame, "./multiRoom.pgm")
        
 


    def showGraphView(self):
        graphFrame = Frame(self.master, padx=25)
        graphFrame.grid(row=0, column = 1)
        title = Label(graphFrame,text="地图",font=("Times", "16", "bold italic"))
        title.pack()


        self.map=Map(graphFrame,"./room_grey.pgm")


    def showCommand(self):
        scaleFrame = Frame(self.master,padx=5, pady=50)
        scaleFrame.grid(row=1, column = 1,sticky=W+N)
        scaleLabel = Label(scaleFrame, text="速度:",font=("Times", "16", "bold italic"))
        scaleLabel.grid(row=0, column=0, sticky=E+S)
        self.scale = Scale(scaleFrame, orient=HORIZONTAL, from_= 0, to=20, length=400)
        self.scale.grid(row=0, column=1)
        self.scale.bind("<ButtonRelease-1>", self.updateRobotVelocity)
        placeholder = Label(scaleFrame, text="  ", width=10, anchor=E)
        placeholder.grid(row = 1, column =2, columnspan=2, sticky= W)

        pauseButtion = Button(scaleFrame,text = '暂停',command = self.__pause)
        pauseButtion.grid(row=2,column=0,sticky=W+S)
        restartButtion = Button(scaleFrame,text = '初始化',command = self.__restart)
        restartButtion.grid(row=2,column=1,sticky=W+S)
        

    def __pause(self):
        self.scale.set(0)
        self.robot.setVelocity(self.scale.get())

   

    def showDashBoard(self):
        frame = Frame(self.master, pady=50)
        frame.grid(row=1, column = 0)
        dashFrame = LabelFrame(frame, text="统计数据", 
            padx=10, pady=10, font=("Times", "16", "bold italic"))
        dashFrame.pack()

        label = Label(dashFrame, text="覆盖率:", width=10, anchor=E)
        label.grid(row = 0, column =0, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.coverageRate, width=10,anchor=W)
        value.grid(row =0, column=1, sticky=W)

        label = Label(dashFrame, text="重复率:", width=10, anchor=E)
        label.grid(row = 0, column =2, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.repeatedRate, width=10, anchor=W)
        value.grid(row =0, column=3, sticky=W)

        placeholder = Label(dashFrame, text="  ", width=10, anchor=E)
        placeholder.grid(row = 1, column =2, columnspan=4, sticky= W)

        label = Label(dashFrame, text="位置:", width=10, anchor=E)
        label.grid(row = 2, column =0, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.robotPos, width=10,anchor=W)
        value.grid(row =2, column=1, sticky=W)

        label = Label(dashFrame, text="前距离:", width=10, anchor=E)
        label.grid(row = 2, column =2, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.head, width=10,anchor=W)
        value.grid(row =2, column=3, sticky=W)

        label = Label(dashFrame, text="左距离:", width=10, anchor=E)
        label.grid(row = 3, column =0, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.left, width=10, anchor=W)
        value.grid(row =3, column=1, sticky=W)

        label = Label(dashFrame, text="右距离:", width=10, anchor=E)
        label.grid(row = 3, column =2, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.right, width=10, anchor=W)
        value.grid(row =3, column=3, sticky=W)

        label = Label(dashFrame, text="状态:", width=10, anchor=E)
        label.grid(row = 4, column =0, sticky= W)
        value = Label(dashFrame, textvariable=self.statistic.robotState, width=20, anchor=W, foreground="blue",font=("Times", "12", "bold italic"))
        value.grid(row =4, column=1, sticky=W)


    def updateRobotVelocity(self,event):
        self.robot.setVelocity(self.scale.get())


    def run(self):
        self.robot.periodicUpdateStatistic()


root = Tk()
root.title("仿真在线路径规划")
app = App(root)

root.mainloop()



