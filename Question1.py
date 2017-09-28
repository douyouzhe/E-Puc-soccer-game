# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 21:08:24 2016

@author: PEARL LIM
"""

#Import Libraries:
import vrep #import library for VREP API
import time #time library
import math
import binascii
import numpy as np #array library
import matplotlib.pyplot as mpl #used for image plotting

print 'Python program started'
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19998,True,True,5000,5) #Timeout=5000ms, Threadcycle=5ms  
if clientID!=-1: 
    print 'Connected to V-REP'    
else:
    print 'Failed connecting to V-REP'
    vrep.simxFinish(clientID)
    
    
if clientID!=-1: 
    
    #ball position
    ErrorCode, ballPos = vrep.simxGetObjectHandle(clientID,'Ball',vrep.simx_opmode_oneshot_wait)
    ErrorCode, ballXYZ = vrep.simxGetObjectPosition(clientID,ballPos,-1,vrep.simx_opmode_streaming)
    #ErrorCode, ballPos = vrep.simxGetObjectPosition (clientID,ball,-1,vrep.simx_opmode_streaming)
    #ErrorCode, ballPos = vrep.simxGetObjectPosition (clientID,ball,-1,vrep.simx_opmode_buffer)

    #motors here
    ErrorCode, LeftJointHandle = vrep.simxGetObjectHandle(clientID,'Blue1_leftJoint',vrep.simx_opmode_oneshot_wait)  #Left motor
    ErrorCode, RightJointHandle = vrep.simxGetObjectHandle(clientID,'Blue1_rightJoint',vrep.simx_opmode_oneshot_wait) #Right motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) 
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)   
    
    #get Blue1's position and orientation
    ErrorCode, bluePos = vrep.simxGetObjectHandle(clientID,'Blue1',vrep.simx_opmode_oneshot_wait)
    ErrorCode, blueXYZ = vrep.simxGetObjectPosition(clientID,bluePos,-1,vrep.simx_opmode_streaming)
    ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming)

    
    #functions
    def Blue1coord():

        ErrorCode, blueXYZ = vrep.simxGetObjectPosition(clientID,bluePos,-1,vrep.simx_opmode_streaming)
        #according to blue1's direction
        if blueXYZ[0] < 0 : # -0.45 to 0.45 (right to left) change to (0 to 90)
            x = int((0.45 - ((blueXYZ[0])*(-1)))*100)
        else:
            x = int((0.45 + (blueXYZ[0]))*100)

        if blueXYZ[1] < 0: # -0.7 to 0.7 (top to bottom) change to (0 to 140)
            y = int((0.7 - ((blueXYZ[1])*(-1)))*100)
        else:
            y = int((0.7 + (blueXYZ[1]))*100)
        #x = blueXYZ[0]
        #y = blueXYZ[1]

        return x, y

    def Ballcoord():

        ErrorCode, ballXYZ = vrep.simxGetObjectPosition(clientID,ballPos,-1,vrep.simx_opmode_streaming)
        #according to ball's direction
        if ballXYZ[0] < 0 : # -0.45 to 0.45 (right to left) change to (0 to 90)
            x = int((0.45 - ((ballXYZ[0])*(-1)))*100)
        else:
            x = int((0.45 + (ballXYZ[0]))*100)

        if ballXYZ[1] < 0: # -0.7 to 0.7 (top to bottom) change to (0 to 140)
            y = int((0.7 - ((ballXYZ[1])*(-1)))*100)
        else:
            y = int((0.7 + (ballXYZ[1]))*100)
        #x = ballXYZ[0]
        #y = ballXYZ[1]

        return x, y

    def TurnRightFromLeft():

        ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming)
        heading = angles[2]*180/math.pi
        while (heading>115 or heading<0):
            LeftVelocity = 505
            RightVelocity = -505
            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
            ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming)
            heading = angles[2]*180/math.pi
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed

    def TurnLeftFromRight():
        ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming)
        heading = angles[2]*180/math.pi
        while (heading>70 or heading<-115):
                LeftVelocity = -505
                RightVelocity = 505
                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming)
                heading = angles[2]*180/math.pi
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed


    #initial declarations
    LeftVelocity = 0
    RightVelocity = 0
    t = time.time()
    end = 0
    check = 0

    #ballX,ballY = Ballcoord()

    while(end == 0):

        blueX,blueY = Blue1coord()
        ballX,ballY = Ballcoord()

        while (0<ballX<44): #ball is on the right
            ballX,ballY = Ballcoord()
            if (0<ballX<44):
                ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming )
                heading = angles[2]*180/math.pi
                if (heading>115 or heading<0):
                    TurnRightFromLeft()

                elif (37<ballX<44):
                    ballX,ballY = Ballcoord()
                    if(37<ballX<44):
                        if (blueX>ballX):
                            if (blueX>25):
                                LeftVelocity = 400
                                RightVelocity = 400
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                            else:
                                LeftVelocity = 0
                                RightVelocity = 0
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                        else:
                            LeftVelocity = -400
                            RightVelocity = -400
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait)
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)

                elif (32<ballX<38):
                    if (blueX>ballX):
                        if (blueX>25):
                            LeftVelocity = 500
                            RightVelocity = 500
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                        else:
                            LeftVelocity = 0
                            RightVelocity = 0
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                    else:
                        LeftVelocity = -500
                        RightVelocity = -500
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait)
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)

                elif (ballX<33):
                    if (blueX>ballX):
                        if (blueX>25):
                            LeftVelocity = 600
                            RightVelocity = 600
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                        else:
                            LeftVelocity = 0
                            RightVelocity = 0
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                    else:
                        LeftVelocity = -600
                        RightVelocity = -600
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait)
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)

            blueX,blueY = Blue1coord()
            ballX,ballY = Ballcoord()


        while (45<ballX<91): #ball is on the left
            ballX,ballY = Ballcoord()
            if(45<ballX<91):
                ErrorCode, angles= vrep.simxGetObjectOrientation(clientID,bluePos,-1,vrep.simx_opmode_streaming )
                heading = angles[2]*180/math.pi
                if (heading<-115 or heading>70):
                    TurnLeftFromRight()

                elif (56<ballX):
                    if (blueX<ballX):
                        if (blueX<60):
                            LeftVelocity = 600
                            RightVelocity = 600
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                        else:
                            LeftVelocity = 0
                            RightVelocity = 0
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                    else:
                        LeftVelocity = -600
                        RightVelocity = -600
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait)
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)

                elif (52<ballX<57):
                    if (blueX<ballX):
                        if (blueX<60):
                            LeftVelocity = 500
                            RightVelocity = 500
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                        else:
                            LeftVelocity = 0
                            RightVelocity = 0
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                    else:
                        LeftVelocity = -500
                        RightVelocity = -500
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait)
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)

                elif (ballX<53):
                    ballX,ballY = Ballcoord()
                    if(ballX<53):
                        if (blueX<ballX):
                            if (blueX<60):
                                LeftVelocity = 400
                                RightVelocity = 400
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                            else:
                                LeftVelocity = 0
                                RightVelocity = 0
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                                ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left
                        else:
                            LeftVelocity = -400
                            RightVelocity = -400
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait)
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
                            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait)

            blueX,blueY = Blue1coord()
            ballX,ballY = Ballcoord()


        if (43<ballX<46): #centre
            LeftVelocity = 0
            RightVelocity = 0
            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
            ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left

        blueX,blueY = Blue1coord()
        ballX,ballY = Ballcoord()

        if (abs(blueY-ballY)<12): #Backup to ensure that Robot will not go crazy after saving goal
            check = 1
        if check == 1:
            ballX,oldballY = Ballcoord()
            time.sleep(1)
            ballX,ballY = Ballcoord()
            if (oldballY == ballY):
                end = 1


    LeftVelocity = 0
    RightVelocity = 0
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left


vrep.simxFinish(clientID)    
print 'Program ended'         