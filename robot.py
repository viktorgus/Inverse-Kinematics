from graphics import *
from math import *
from numpy import *
import time


winX=1000
winY=500
baseX=500
baseY=70


def main():

    win = GraphWin("Kinematics Test", winX,winY)
    win.setCoords(0,0,winX,winY)
    win.setBackground(color_rgb(0,0,0))
    robot = Robot(pi/2,0,0,125,125,185)
    robot.drawAll(win)
    i=0

    while (True):
        mousePoint=win.getMouse()
        x=mousePoint.x
        y=mousePoint.y
        while not robot.feasiblePoint(x,y): 
            print("point out of robot range, try again")
            mousePoint=win.getMouse()
            x=mousePoint.x
            y=mousePoint.y
        robot.iterateToPoint([x,y],win)
        robot.drawAll(win)
        angleInfo = Text(Point(winX-200,winY-50),"Angles    a: " + str(round(degrees(robot.angles[0]))) + " b: " + str(round(degrees(robot.angles[1]))) + " c: " + str(round(degrees(robot.angles[2]))))
        angleInfo.setTextColor("white")
        angleInfo.draw(win)
        robot.angles[0]=0
        robot.angles[1]=0
        robot.angles[2]=0
        robot.updateSegPos()


def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

class Robot():
    absAngles=[0,0,0]
    angles=[0,0,0]
    aCords=[[baseX,baseY],[0,0],[0,0],[0,0]]
    lengths = [125,125,185]

    def __init__(self,a,b,c,lenA,lenB,lenC):
        self.lengths[0]=lenA
        self.lengths[1]=lenB
        self.lengths[2]=lenC
        self.angles[0]=a
        self.angles[1]=b
        self.angles[2]=c  
        self.updateSegPos()

    def updateSegPos(self):
        self.absAngles[0]=self.angles[0]
        self.absAngles[1]=self.angles[0]+self.angles[1]-pi/2
        self.absAngles[2]=self.angles[0]+self.angles[1]+self.angles[2]-pi
        i = 0
        while i<=len(self.aCords)-2:
            self.aCords[i+1][0]=self.aCords[i][0]+cos(self.absAngles[i])*self.lengths[i]
            self.aCords[i+1][1]=self.aCords[i][1]+sin(self.absAngles[i])*self.lengths[i]
            i+=1
        #print("Updated angles to -  A: " + str(degrees(self.absAngles[0])) + "   B: " + str(degrees(self.absAngles[1])) + "  C:  " + str(degrees(self.absAngles[2])))


    def getR(self):
        return self.lengths[2]*cos(self.absAngles[2])+self.lengths[1]*cos(self.absAngles[1])+self.lengths[0]*cos(self.absAngles[0])+baseX

    def getZ(self):
        return self.lengths[2]*sin(self.absAngles[2])+self.lengths[1]*sin(self.absAngles[1])+self.lengths[0]*sin(self.absAngles[0])+baseY


    def validateR(self):
        return self.getR()==self.aCords[3][0]

    def getJacobian(self):
        drda=self.lengths[2]*sin(self.angles[0]+self.angles[1]+self.angles[2])+self.lengths[1]*cos(self.angles[0]+self.angles[1])-self.lengths[0]*sin(self.angles[0])
        drdb=self.lengths[2]*sin(self.angles[0]+self.angles[1]+self.angles[2])+self.lengths[1]*cos(self.angles[0]+self.angles[1])
        drdc=self.lengths[2]*sin(self.angles[0]+self.angles[1]+self.angles[2])
        dzda=-self.lengths[2]*cos(self.angles[0]+self.angles[1]+self.angles[2])+self.lengths[1]*sin(self.angles[0]+self.angles[1])+self.lengths[0]*cos(self.angles[0])
        dzdb=-self.lengths[2]*cos(self.angles[0]+self.angles[1]+self.angles[2])+self.lengths[1]*sin(self.angles[0]+self.angles[1])
        dzdc=-self.lengths[2]*cos(self.angles[0]+self.angles[1]+self.angles[2])
        return [[drda,drdb,drdc],[dzda,dzdb,dzdc]]

    def getPseudoInverse(self):
        jacobian = self.getJacobian()
        jacobianTranspose = transpose(jacobian)
        jjT = matmul(jacobian,jacobianTranspose)
        inversejjT = linalg.inv(jjT)
        return matmul(jacobianTranspose,inversejjT)

    def adda(self,a):
        self.angles[0]+=a
        self.updateSegPos()
        

    def addb(self,b):
        self.angles[1]+=b
        self.updateSegPos()    
        

    def addc(self,c):
        self.angles[2]+=c
        self.updateSegPos()

    def feasiblePoint(self,x,y):
        r=square(x-baseX)+square(y-baseY)
        return r>square(self.lengths[0])+square(self.lengths[0]-self.lengths[1]) and r<square(self.lengths[0]+self.lengths[1]+self.lengths[2]) and x-baseX>0

    def iterateToPoint(self,point,win):
        start = time.time()
        goalPoint=Circle(Point(point[0],point[1]),3)
        goalPoint.setWidth(5)
        goalPoint.setOutline(color_rgb(255,255,0)) 
        label = Text(Point(winX-150,winY-50),"Searching for point...")
        label.setTextColor("white")
        while (square(point[0]-self.getR())>5) or (square(point[1]-self.getZ())>5):
            self.updateSegPos()
            v = [point[0]-self.getR(),point[1]-self.getZ()]
            
            pseudo = self.getPseudoInverse()
            dAngles = matmul(pseudo,v)
            if not ((dAngles[0]+self.angles[0]>pi) or (dAngles[0]+self.angles[0]<0)):
                self.adda(dAngles[0])
            if not ((dAngles[1]+self.angles[1]>pi-0.262) or (dAngles[1]+self.angles[1]<0+0.262)):
                self.addb(dAngles[1])
            if not ((dAngles[2]+self.angles[2]>pi) or (dAngles[2]+self.angles[2]<0)):
                self.addc(dAngles[2])
            label.draw(win)
            goalPoint.draw(win)
            self.drawAll(win)
            now = time.time()
            if (now-start)>1:
                print("iteration timed out")
                break
            

    def drawAll(self,win):
        clear(win)
        lines = [None,None,None]
        cirkles = [None,None,None]
        i=0
        while i<=len(lines)-1: 
            lines[i]=Line(Point(self.aCords[i][0],self.aCords[i][1]),Point(self.aCords[i+1][0],self.aCords[i+1][1]))
            lines[i].setOutline(color_rgb(255,69,0))
            lines[i].setWidth(5)
            cirkles[i]=Circle(Point(self.aCords[i][0],self.aCords[i][1]),20)
            cirkles[i].setOutline(color_rgb(255,69,0)) 
            cirkles[i].setWidth(5)
            lines[i].draw(win)
            cirkles[i].draw(win)
            i+=1
       
main()
