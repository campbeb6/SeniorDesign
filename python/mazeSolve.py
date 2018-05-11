import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')

from picamera import PiCamera

import math
import copy
import time
import cv2
import numpy as np

RESX = 640
RESY = 480

low_gr = np.array([30,40,80])
upr_gr = np.array([50,255,255])

camera = PiCamera()
camera.resolution = (RESX,RESY)


class MazeSolver(object):

    #initializer
    def __init__(self, xCells, yCells):
        self.xCells = xCells
        self.yCells = yCells

        
    def getMaze(self):
        self.img = self.capture()
        self.maskImg = self.mask(self.img)
        self.sizes = self.getSize(self.maskImg)
        self.centers = self.findCenters()
        self.inMaze = self.findWalls(self.maskImg)
        
    def solveMaze(self,carX,carY):     
        
        exits = self.closeMaze()
        
        end1 = exits.pop()
        end2 = exits.pop()
        print(end1)
        print(end2)
        start = self.findStart(carX,carY)
            
        print(start)

        inMaze1 = copy.deepcopy(self.inMaze)
        inMaze2 = copy.deepcopy(self.inMaze)

        self.path1 = self.fillMaze(inMaze1,start,end1)
        self.path2 = self.fillMaze(inMaze2,start,end2)

        self.route1 = self.findPath(start,end1,self.path1)
        self.route2 = self.findPath(start,end2,self.path2)

        if(start == end1):
            self.finalPath = self.route2
        elif(start == end2):
            self.finalPath = self.route1
        elif(len(self.route1) <= len(self.route2)):
            self.finalPath = self.route1
        else:
            self.finalPath = self.route2
        
        self.finalCoords =  self.getCoords()


################################################################################

    def capture(self):
        print("capturing image")
        time.sleep(1)
        camera.capture('pic2.jpg')
        img = cv2.imread('pic2.jpg')
        img2 = cv2.resize(img, (RESX,RESY))
        camera.close()
        return img2

#############################################################################

    def mask(self, img):
        print("creating mask")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blur = cv2.medianBlur(hsv, 5)
        mask = cv2.inRange(blur, low_gr, upr_gr)
        cv2.bitwise_not(mask, mask)

        return mask

###############################################################################

    def getSize(self, mask):
        print("getting size")
        self.yMin = RESY-1
        self.yMax = 0
        self.xMin = RESX-1
        self.xMax = 0

        #find top 
        for x in range(RESX):
            for y in range(RESY):
                if(mask[y,x] == 0):
                    if y < self.yMin:
                        self.yMin = y
                    break

        #find bottom
        for x in range(RESX):
            for y in range(RESY):
                y1 = RESY-y-1
                if(mask[y1,x] == 0):
                    if y1 > self.yMax:
                        self.yMax = y1
                    break

        #find left
        for y in range(RESY): 
            for x in range(RESX):
                if(mask[y,x] == 0):
                    if x < self.xMin:
                        self.xMin = x
                    break

        #find right
        for y in range(RESY):
            for x in range(RESX):
                x1 = RESX-x-1
                if(mask[y,x1] == 0):
                    if x1 > self.xMax:
                        self.xMax = x1
                    break
        #get values of left, right, bottom and top
        #get height and width of maze in pixels
        self.height = self.yMax - self.yMin
        self.width = self.xMax - self.xMin
        self.cellWidth = self.width/self.xCells
        self.cellHeight = self.height/self.yCells
        self.xSpace = self.cellWidth/2
        self.ySpace = self.cellHeight/2

##############################################################################
    
    def findCenters(self):
        
        print("Finding centers")
        centers = np.zeros((self.xCells,self.yCells,2))
        
        #find centers of cells
        for x in range(self.xCells):
            for y in range(self.yCells):
                centers[x,y,0] = self.yMin + self.ySpace + int((self.cellHeight*y))
                centers[x,y,1] = self.xMin + self.xSpace + int((self.cellWidth*x))
        return centers

#############################################################################

    def findWalls(self,mask):
        xS = self.xSpace + 10
        yS = self.ySpace + 10
        
        output = np.zeros((self.xCells,self.yCells))
        
        #search up
        for x in range(self.xCells):
            for y in range(self.yCells):
                for z in range(yS):
                    if(mask[int(self.centers[x,y,0])-z,int(self.centers[x,y,1])] == 0):
                        output[x,y] = output[x,y] + 8
                        break
        
        #search down
        for x in range(self.xCells):
            for y in range(self.yCells):
                for z in range(yS):
                    if(mask[int(self.centers[x,y,0])+z,int(self.centers[x,y,1])] == 0):
                        output[x,y] = output[x,y] + 4
                        break

        #search right
        for x in range(self.xCells):
            for y in range(self.yCells):
                for z in range(xS):
                    if(mask[int(self.centers[x,y,0]),int(self.centers[x,y,1])+z] == 0):
                        output[x,y] = output[x,y] + 2
                        break

        #search left
        for x in range(self.xCells):
            for y in range(self.yCells):
                for z in range(xS):
                    if(mask[int(self.centers[x,y,0]),int(self.centers[x,y,1])-z] == 0):
                        output[x,y] = output[x,y] + 1
                        break
        
        return output

###########################################################################    
    
    def fillMaze(self, inMaze, start, end):
        print("dead end filling")
        found = True
        outMaze = inMaze
        while found:
            found = False
            for x in range(self.xCells):
                for y in range(self.yCells):
                    if(not ((start[0]==x and start[1]==y)or(end[0]==x and end[1]==y))):
                        if(outMaze[x,y] == 7):
                            outMaze[x,y] = 15
                            outMaze[x,y-1] += 4
                            found = True
                        #no bottom
                        elif(outMaze[x,y] == 11):
                            outMaze[x,y] = 15
                            outMaze[x,y+1] += 8
                            found = True

                        #no right wall
                        elif(outMaze[x,y] == 13):
                            outMaze[x,y] = 15
                            outMaze[x+1,y] += 1
                            found = True

                        #no left wall
                        elif(outMaze[x,y] == 14):
                            outMaze[x,y] = 15
                            outMaze[x-1,y] +=2
                            found = True

        return outMaze

###############################################################################

    def findPath(self,start,end,pathFinder):
        print("getting path")
        x = start[0]
        y = start[1]

        path = []
        pathFound = False

        while not pathFound:
            path.append([x,y])
            currentCell = pathFinder[x,y]
            if (x==end[0] and y==end[1]):
                pathFound = True

            else:
                #no top
                if(currentCell == 7):
                    pathFinder[x,y]=15
                    pathFinder[x,y-1]+=4
                    y-=1
                #no bottom
                elif(currentCell == 11):
                    pathFinder[x,y]=15
                    pathFinder[x,y+1]+=8
                    y+=1
                #no right side
                elif(currentCell == 13):
                    pathFinder[x,y]=15
                    pathFinder[x+1,y]+=1
                    x+=1
                #no left side
                elif(currentCell == 14):
                    pathFinder[x,y]=15
                    pathFinder[x-1,y]+=2
                    x-=1
        
        return path

#############################################################################

    def getCoords(self):
        print("getting coordinates")
        coords = []
        for spot in self.finalPath:
            coords.append([int(self.centers[spot[0],spot[1],1]),int(self.centers[spot[0],spot[1],0])])

        return coords 



##########################################################################

    def closeMaze(self):
        
        openings = []
        #close all openings on left side of maze, and save their location
        x = 0
        for y in range(self.yCells):
            if(self.inMaze[x,y]==6 or self.inMaze[x,y]==10 or self.inMaze[x,y]==12 or self.inMaze[x,y]==8 or self.inMaze[x,y]==4 or self.inMaze[x,y]==2):
                self.inMaze[x,y]+=1
                openings.append([x,y])
                print("found left exit")
        
        #close all openings on right side of maze
        x = self.xCells-1   
        for y in range(self.yCells):
            if(self.inMaze[x,y]==5 or self.inMaze[x,y]==9 or self.inMaze[x,y]==12 or self.inMaze[x,y]==8 or self.inMaze[x,y]==4 or self.inMaze[x,y]==1):
                self.inMaze[x,y]+=2
                openings.append([x,y])
                print("found right exit")

        #close openings on top
        y = 0
        for x in range(self.xCells):
            if(self.inMaze[x,y]==3 or self.inMaze[x,y]==5 or self.inMaze[x,y]==6 or self.inMaze[x,y]==4 or self.inMaze[x,y]==2 or self.inMaze[x,y]==1):
                self.inMaze[x,y]+=8
                openings.append([x,y])
                print("found top exit")

        #close openings on bottom
        y = self.yCells-1
        for x in range(self.xCells):
            if(self.inMaze[x,y]==3 or self.inMaze[x,y]==9 or self.inMaze[x,y]==10 or self.inMaze[x,y]==8 or self.inMaze[x,y]==2 or self.inMaze[x,y]==1):
                self.inMaze[x,y]+=4
                openings.append([x,y])
                print("found bottom exit")


        return openings

###############################################################################3

    def findStart(self,carX,carY):
        minDist = 10000
        start = [0,0]
        for x in range(self.xCells):
            for y in range(self.yCells):
                x2 = int(self.centers[x,y,1])
                y2 = int(self.centers[x,y,0])
                
                xDiff = carX-x2
                yDiff = carY-y2
                
                dist = math.sqrt(xDiff**2 + yDiff**2)
                #print(dist)
                if dist < minDist:
                    minDist = dist
                    start = [x,y]

        return start


