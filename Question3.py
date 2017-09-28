# -*- coding: utf-8 -*-
"""

Team Name: Arsenal FC
Team Members: Youzhe, Max, Pearl, Zi Liang
Date: 6 March 2016
Version: Version 1.0

"""
#Import Libraries:
import vrep 
import time 
import math
import numpy as np #array library
import matplotlib.pyplot as mpl #used for image plotting

speed = 2
grid = [[0 for x in range(256)] for x in range(128)]
grid_arr = np.array(grid, dtype=np.uint16)

def rotationOnSpotCW1(speed):
   rotationSpeed = speed #adjust the speed here
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle1,rotationSpeed,vrep.simx_opmode_oneshot_wait)
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle1,-rotationSpeed,vrep.simx_opmode_oneshot_wait)
   return

def rotationOnSpotCCW1(speed):
   rotationSpeed = speed #adjust the speed here
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle1,-rotationSpeed,vrep.simx_opmode_oneshot_wait)
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle1,rotationSpeed,vrep.simx_opmode_oneshot_wait)
   return

def rotationOnSpotCW2(speed):
   rotationSpeed = speed #adjust the speed here
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle2,rotationSpeed,vrep.simx_opmode_oneshot_wait)
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle2,-rotationSpeed,vrep.simx_opmode_oneshot_wait)
   return

def rotationOnSpotCCW2(speed):
   rotationSpeed = speed #adjust the speed here
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle2,-rotationSpeed,vrep.simx_opmode_oneshot_wait)
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle2,rotationSpeed,vrep.simx_opmode_oneshot_wait)
   return

def rotationOnSpotCW3(speed):
   rotationSpeed = speed #adjust the speed here
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle3,rotationSpeed,vrep.simx_opmode_oneshot_wait)
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle3,-rotationSpeed,vrep.simx_opmode_oneshot_wait)
   return

def rotationOnSpotCCW3(speed):
   rotationSpeed=speed #adjust the speed here
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle3,-rotationSpeed,vrep.simx_opmode_oneshot_wait)
   ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle3,rotationSpeed,vrep.simx_opmode_oneshot_wait)
   return

def RotateOnSpot(desiredAngle, robotID):
   allowance = 1.5
   desiredAngle = desiredAngle
   mytime = time.time()
   runningtime = 0
   if (robotID == 1):
       ErrorCode, angles = vrep.simxGetObjectOrientation(clientID, Red1_Orientation, -1, vrep.simx_opmode_buffer)
       currentAngle = angles[2]*180/math.pi
       while ((abs(currentAngle - desiredAngle)) > allowance) and runningtime < 0.3:#adjust for allowance
           runningtime = time.time() - mytime
           ErrorCode, angles = vrep.simxGetObjectOrientation(clientID, Red1_Orientation, -1, vrep.simx_opmode_buffer)
           currentAngle = angles[2]*180/math.pi
           speed = abs(currentAngle - desiredAngle)*0.05
           if currentAngle >= 179 or currentAngle <= -179:
               if desiredAngle < 0:
                   currentAngle = -179
               else:
                   currentAngle = 179
           if desiredAngle > currentAngle:
               rotationOnSpotCCW1(speed)#adjust for resolution
           else:
               rotationOnSpotCW1(speed)#adjust for resolution
       if (abs(currentAngle - desiredAngle) < allowance):
           ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle1,0,vrep.simx_opmode_oneshot_wait)
           ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle1,0,vrep.simx_opmode_oneshot_wait)
           
   if (robotID == 2):
       ErrorCode, angles = vrep.simxGetObjectOrientation(clientID, Red2_Orientation, -1, vrep.simx_opmode_buffer)
       currentAngle = angles[2]*180/math.pi
       while (abs(currentAngle - desiredAngle) > allowance) and runningtime < 0.3:#adjust for allowance
           runningtime = time.time() - mytime           
           speed = abs(currentAngle - desiredAngle)*0.05
           ErrorCode, angles = vrep.simxGetObjectOrientation(clientID, Red2_Orientation, -1, vrep.simx_opmode_buffer)
           currentAngle = angles[2]*180/math.pi
           if (desiredAngle > currentAngle):
               rotationOnSpotCCW2(speed)#adjust for resolution
           else:
               rotationOnSpotCW2(speed)#adjust for resolution
       if (abs(currentAngle - desiredAngle) < allowance):
           ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle2,0,vrep.simx_opmode_oneshot_wait)
           ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle2,0,vrep.simx_opmode_oneshot_wait)
           
   if (robotID == 3):
       ErrorCode, angles = vrep.simxGetObjectOrientation(clientID, Red3_Orientation, -1, vrep.simx_opmode_buffer)
       currentAngle = angles[2]*180/math.pi
       while (abs(currentAngle - desiredAngle) > allowance) and runningtime < 0.3:#adjust for allowance
           runningtime = time.time() - mytime           
           speed = abs(currentAngle - desiredAngle)*0.05
           ErrorCode, angles = vrep.simxGetObjectOrientation(clientID, Red3_Orientation, -1, vrep.simx_opmode_buffer)
           currentAngle = angles[2]*180/math.pi
           if (desiredAngle > currentAngle):
               rotationOnSpotCCW3(speed)#adjust for resolution
           else:
               rotationOnSpotCW3(speed)#adjust for resolution
       if (abs(currentAngle - desiredAngle) < allowance):
           ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle3,0,vrep.simx_opmode_oneshot_wait)
           ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle3,0,vrep.simx_opmode_oneshot_wait)

def Get_Image():
    ErrorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID,CamHandle,0,vrep.simx_opmode_oneshot_wait) # Get image
    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[1],resolution[0],3])
    im_red = im[:,:,0]
    im_red = np.array(im_red, dtype=np.uint16)
    im_red_arr = np.fliplr(im_red)
    for i in range(39,216,1):#i=column
        for j in range(0,128,1):#j=row
            if (im_red_arr[j][i] == 241 and i >= 89 and i <= 99) or (im_red_arr[j][i] == 241 and i >= 157 and i <= 167):                      
                im_red_arr[j][i] = 1000
                for k in range (1,5,1):
                    if im_red_arr[j-k][i] != 1000 and im_red_arr[j-k][i] != 241:
                        im_red_arr[j-k][i] = 1000
                    if im_red_arr[j+k][i] != 1000 and im_red_arr[j+k][i] != 241:
                        im_red_arr[j+k][i] = 1000
                    if im_red_arr[j][i+k] != 1000 and im_red_arr[j][i+k] != 241:
                        im_red_arr[j][i+k] = 1000
                    if im_red_arr[j][i-k] != 1000 and im_red_arr[j][i-k] != 241:
                        im_red_arr[j][i-k] = 1000
            if (j >= 0 and j <= 20) or (j >= 110 and j <= 127):
                im_red_arr[j][i] = 1000
    return im_red_arr    
           
def Move_Robot(LeftVelocity, RightVelocity, Robot):
    if Robot == 1:
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle1,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle1,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
    elif Robot == 2:
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle2,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle2,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
    elif Robot == 3:
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle3,LeftVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle3,RightVelocity*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
    return
            
def Get_Robot_Position(Robot):
    if Robot == 1:    
        ErrorCode, xyz_robot1 = vrep.simxGetObjectPosition(clientID, Red1_Position, -1, vrep.simx_opmode_buffer)
        for i in range(0,3,1):
            xyz_robot1[i] *= 100
        return xyz_robot1
    elif Robot == 2:
        ErrorCode, xyz_robot2 = vrep.simxGetObjectPosition(clientID, Red2_Position, -1, vrep.simx_opmode_buffer)
        for i in range(0,3,1):
            xyz_robot2[i] *= 100
        return xyz_robot2
    elif Robot == 3:
        ErrorCode, xyz_robot3 = vrep.simxGetObjectPosition(clientID, Red3_Position, -1, vrep.simx_opmode_buffer)
        for i in range(0,3,1):
            xyz_robot3[i] *= 100
        return xyz_robot3
        
def Get_Ball_Position():
    ErrorCode, xyz_ball = vrep.simxGetObjectPosition(clientID, BallPosition, -1, vrep.simx_opmode_buffer)        
    for i in range(0,3,1):
        xyz_ball[i] *= 100
    return xyz_ball
    
def GetRobotOrientation(robotID):
    if (robotID == 1):
        ErrorCode, angles_robot1 = vrep.simxGetObjectOrientation(clientID, Red1_Orientation, -1, vrep.simx_opmode_buffer)
        heading = angles_robot1[2]*180/math.pi
    if (robotID == 2):
        ErrorCode, angles_robot2 = vrep.simxGetObjectOrientation(clientID, Red2_Orientation, -1, vrep.simx_opmode_buffer)
        heading = angles_robot2[2]*180/math.pi
    if (robotID == 3):
        ErrorCode, angles_robot3 = vrep.simxGetObjectOrientation(clientID, Red3_Orientation, -1, vrep.simx_opmode_buffer)
        heading = angles_robot3[2]*180/math.pi     
    return heading
    
def robotMoveToThere(robotID,xPos,yPos):
    relativeSpeed = 300
    if (robotID == 1):
        ErrorCode, xyz_robot1 = vrep.simxGetObjectPosition(clientID, Red1_Position, -1, vrep.simx_opmode_buffer)
        if (xPos >= xyz_robot1[0]) and (yPos >= xyz_robot1[1]):
            targetAngle = math.atan2(xPos-xyz_robot1[0],yPos-xyz_robot1[1])
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(0-targetAngle,1)
            flag = 1
            
        if (xPos <= xyz_robot1[0]) and (yPos >= xyz_robot1[1]):
            targetAngle = math.atan2(xyz_robot1[0]-xPos,yPos-xyz_robot1[1])
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(targetAngle,1)        
            flag = 2
            
        if (xPos >= xyz_robot1[0]) and (yPos <= xyz_robot1[1]):
            targetAngle = math.atan2(xPos-xyz_robot1[0],xyz_robot1[1]-yPos)
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(targetAngle-180,1)         
            flag = 3
            
        if (xPos <= xyz_robot1[0]) and (yPos <= xyz_robot1[1]):
            targetAngle = math.atan2(xyz_robot1[0]-xPos,xyz_robot1[1]-yPos)
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(180-targetAngle,1)
            flag = 4
        
        ErrorCode, xyz_robot1 = vrep.simxGetObjectPosition(clientID, Red1_Position, -1, vrep.simx_opmode_buffer)
            
        if (abs(xPos-xyz_robot1[0])>0.02 or abs(yPos-xyz_robot1[1] > 0.02)):
            print "move"
            Move_Robot(relativeSpeed, relativeSpeed, 1)
        else: 
            Move_Robot(0, 0, 1)
            RotateOnSpot(90,1)
            print "stop moving"
        
    if (robotID == 2):
        ErrorCode, xyz_robot2 = vrep.simxGetObjectPosition(clientID, Red2_Position, -1, vrep.simx_opmode_buffer)
        
        if (xPos>=xyz_robot2[0]) and (yPos>=xyz_robot2[1]):
            targetAngle=math.atan2(xPos-xyz_robot2[0],yPos-xyz_robot2[1])
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(0-targetAngle,2)
            
        if (xPos<=xyz_robot2[0]) and (yPos>=xyz_robot2[1]):
            targetAngle=math.atan2(xyz_robot2[0]-xPos,yPos-xyz_robot2[1])
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(targetAngle,2)        
            
        if (xPos>=xyz_robot2[0]) and (yPos<=xyz_robot2[1]):
            targetAngle=math.atan2(xPos-xyz_robot2[0],xyz_robot2[1]-yPos)
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(targetAngle-180,2) 
            
        if (xPos<=xyz_robot2[0]) and (yPos<=xyz_robot2[1]):
            targetAngle=math.atan2(xyz_robot2[0]-xPos,xyz_robot2[1]-yPos)
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(180-targetAngle,2)
        
        ErrorCode, xyz_robot2 = vrep.simxGetObjectPosition(clientID, Red2_Position, -1, vrep.simx_opmode_buffer)            

        if (abs(xPos-xyz_robot2[0]) > 0.02 or abs(yPos-xyz_robot2[1] > 0.02)):
            Move_Robot(relativeSpeed, relativeSpeed, 2)
        else: 
            Move_Robot(0, 0, 2)
            RotateOnSpot(90,2)
        
    if (robotID == 3):
        ErrorCode, xyz_robot3 = vrep.simxGetObjectPosition(clientID, Red3_Position, -1, vrep.simx_opmode_buffer)
        
        if (xPos>=xyz_robot3[0]) and (yPos>=xyz_robot3[1]):
            targetAngle=math.atan2(xPos-xyz_robot3[0],yPos-xyz_robot3[1])
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(0-targetAngle,3)
            flag3 = 1
            
        if (xPos<=xyz_robot3[0]) and (yPos>=xyz_robot3[1]):
            targetAngle=math.atan2(xyz_robot3[0]-xPos,yPos-xyz_robot3[1])
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(targetAngle,3)   
            flag3 = 2
            
        if (xPos>=xyz_robot3[0]) and (yPos<=xyz_robot3[1]):
            targetAngle=math.atan2(xPos-xyz_robot3[0],xyz_robot3[1]-yPos)
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(targetAngle-180,3)  
            flag3 = 3
            
        if (xPos <= xyz_robot3[0]) and (yPos <= xyz_robot3[1]):
            targetAngle = math.atan2(xyz_robot3[0]-xPos,xyz_robot3[1]-yPos)
            targetAngle = targetAngle*180/math.pi
            RotateOnSpot(180-targetAngle,3)  
            flag3 = 4

        ErrorCode, xyz_robot3 = vrep.simxGetObjectPosition(clientID, Red3_Position, -1, vrep.simx_opmode_buffer)
            
        if (abs(xPos-xyz_robot3[0])>0.02 or abs(yPos-xyz_robot3[1] > 0.02)):
            Move_Robot(relativeSpeed, relativeSpeed, 3)
        else:
            Move_Robot(0, 0, 3)
            RotateOnSpot(90,3)

def findCloseOpponent():
    ErrorCode, xyz_bluerobot1 = vrep.simxGetObjectPosition(clientID, Blue1_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, xyz_bluerobot2 = vrep.simxGetObjectPosition(clientID, Blue2_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, xyz_bluerobot3 = vrep.simxGetObjectPosition(clientID, Blue3_Position, -1, vrep.simx_opmode_streaming)
    
    dist1 = math.sqrt(math.pow(xyz_bluerobot1[0],2) + math.pow(xyz_bluerobot1[1]+0.7,2))
    dist2 = math.sqrt(math.pow(xyz_bluerobot2[0],2) + math.pow(xyz_bluerobot2[1]+0.7,2))
    dist3 = math.sqrt(math.pow(xyz_bluerobot3[0],2) + math.pow(xyz_bluerobot3[1]+0.7,2))
    
    opponentID = 1
    smallest = dist1
    if dist2 < smallest:
        opponentID = 2
        smallest = dist2
    if dist3 < smallest:
        opponentID = 3
        smallest = dist3
    return opponentID
 
def findgoalkeeper():
    ErrorCode, xyz_bluerobot1 = vrep.simxGetObjectPosition(clientID, Blue1_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, xyz_bluerobot2 = vrep.simxGetObjectPosition(clientID, Blue2_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, xyz_bluerobot3 = vrep.simxGetObjectPosition(clientID, Blue3_Position, -1, vrep.simx_opmode_streaming)
    
    dist1 = math.sqrt(math.pow(xyz_bluerobot1[0],2) + math.pow(xyz_bluerobot1[1]-0.7,2))
    dist2 = math.sqrt(math.pow(xyz_bluerobot2[0],2) + math.pow(xyz_bluerobot2[1]-0.7,2))
    dist3 = math.sqrt(math.pow(xyz_bluerobot3[0],2) + math.pow(xyz_bluerobot3[1]-0.7,2))
    
    goalKeeperID = 1
    smallest = dist1
    if dist2 < smallest:
        goalKeeperID = 2
        smallest = dist2
    if dist3 < smallest:
        goalKeeperID = 3
        smallest = dist3
    return goalKeeperID
    

print 'Python program started'
vrep.simxFinish(-1)
#clientID=vrep.simxStart('192.168.1.3',19998,True,True,5000,5)
clientID=vrep.simxStart('127.0.0.1',19998,True,True,5000,5)
if clientID!=-1: 
    print 'Connected to V-REP'
else:
    print 'Failed connecting to V-REP'
    vrep.simxFinish(clientID)
     
t = time.time()
if clientID!=-1:
    #handles for camera and ball
    ErrorCode, CamHandle = vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait); #Camera
    ErrorCode, BallPosition = vrep.simxGetObjectHandle(clientID,'Ball',vrep.simx_opmode_oneshot_wait) # Position
    ErrorCode, xyz_ball = vrep.simxGetObjectPosition(clientID, BallPosition, -1, vrep.simx_opmode_streaming)
    ErrorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID,CamHandle,0,vrep.simx_opmode_oneshot_wait);
    
    ErrorCode, Blue1_Position = vrep.simxGetObjectHandle(clientID,'Blue1',vrep.simx_opmode_oneshot_wait) # Position
    ErrorCode, Blue2_Position = vrep.simxGetObjectHandle(clientID,'Blue2',vrep.simx_opmode_oneshot_wait) # Position
    ErrorCode, Blue3_Position = vrep.simxGetObjectHandle(clientID,'Blue3',vrep.simx_opmode_oneshot_wait) # Position

    #Get handles robot 1
    ErrorCode, LeftJointHandle1 = vrep.simxGetObjectHandle(clientID,'Red1_leftJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, RightJointHandle1 = vrep.simxGetObjectHandle(clientID,'Red1_rightJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Red1_Position = vrep.simxGetObjectHandle(clientID,'Red1',vrep.simx_opmode_oneshot_wait) # Position
    ErrorCode, Red1_Orientation = vrep.simxGetObjectHandle(clientID,'Red1',vrep.simx_opmode_oneshot_wait) #Orientation
    ErrorCode, xyz_robot1 = vrep.simxGetObjectPosition(clientID, Red1_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, angles_robot1 = vrep.simxGetObjectOrientation(clientID, Red1_Orientation, -1, vrep.simx_opmode_streaming)
    #Get handles for robot 2   
    ErrorCode, LeftJointHandle2 = vrep.simxGetObjectHandle(clientID,'Red2_leftJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, RightJointHandle2 = vrep.simxGetObjectHandle(clientID,'Red2_rightJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Red2_Position = vrep.simxGetObjectHandle(clientID,'Red2',vrep.simx_opmode_oneshot_wait) # Position
    ErrorCode, Red2_Orientation = vrep.simxGetObjectHandle(clientID,'Red2',vrep.simx_opmode_oneshot_wait) #Orientation
    ErrorCode, xyz_robot2 = vrep.simxGetObjectPosition(clientID, Red2_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, angles_robot2 = vrep.simxGetObjectOrientation(clientID, Red2_Orientation, -1, vrep.simx_opmode_streaming)
    #Get handles for robot 3
    ErrorCode, LeftJointHandle3 = vrep.simxGetObjectHandle(clientID,'Red3_leftJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, RightJointHandle3 = vrep.simxGetObjectHandle(clientID,'Red3_rightJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Red3_Position = vrep.simxGetObjectHandle(clientID,'Red3',vrep.simx_opmode_oneshot_wait) # Position
    ErrorCode, Red3_Orientation = vrep.simxGetObjectHandle(clientID,'Red3',vrep.simx_opmode_oneshot_wait) #Orientation
    ErrorCode, xyz_robot3 = vrep.simxGetObjectPosition(clientID, Red3_Position, -1, vrep.simx_opmode_streaming)
    ErrorCode, angles_robot3 = vrep.simxGetObjectOrientation(clientID, Red3_Orientation, -1, vrep.simx_opmode_streaming)
    
    step1 = False
    strategy = 1
    flag = 1
    
    while (1):
        if strategy == 1:
            if not step1:
                Move_Robot(1000,1050,1)
                Move_Robot(-750,-750,2)
                Move_Robot(-750,-750,3)
                time.sleep(1)
                Move_Robot(0,0,1)
                Move_Robot(0,0,2)
                Move_Robot(0,0,3)
                step1 = True
            
            ErrorCode, xyz_ball = vrep.simxGetObjectPosition(clientID, BallPosition, -1, vrep.simx_opmode_buffer)
            ErrorCode, xyz_robot3 = vrep.simxGetObjectPosition(clientID, Red3_Position, -1, vrep.simx_opmode_buffer)
            ErrorCode, xyz_robot1 = vrep.simxGetObjectPosition(clientID, Red1_Position, -1, vrep.simx_opmode_buffer)
          
            ErrorCode, xyz_bluerobot1 = vrep.simxGetObjectPosition(clientID, Blue1_Position, -1, vrep.simx_opmode_streaming)
            ErrorCode, xyz_bluerobot2 = vrep.simxGetObjectPosition(clientID, Blue2_Position, -1, vrep.simx_opmode_streaming)
            ErrorCode, xyz_bluerobot3 = vrep.simxGetObjectPosition(clientID, Blue3_Position, -1, vrep.simx_opmode_streaming)
    
            if xyz_ball[0] > 0.17:
                xyz_ball[0] = 0.17
            if xyz_ball[0] < -0.17:
                xyz_ball[0] = -0.17
            robotMoveToThere(3,xyz_ball[0],-0.7)
            if xyz_robot3[1] > -0.4 and xyz_robot3[0] > 0:
                robotMoveToThere(3,-0.3,-0.7)
            if xyz_robot3[1] > -0.4 and xyz_robot3[0] < 0:
                robotMoveToThere(3,0.3,-0.7)
                
            chooseopponent = findCloseOpponent() #defender hogs onto opponent striker
            if chooseopponent == 1:
                robotMoveToThere(2,xyz_bluerobot1[0], -0.5)
            if chooseopponent == 2:
                robotMoveToThere(2,xyz_bluerobot2[0], -0.5)
            if chooseopponent == 3:
                robotMoveToThere(2,xyz_bluerobot3[0], -0.5)
                
            if xyz_ball[1] >= 0:
                robotMoveToThere(1,xyz_ball[0],xyz_ball[1]-0.05) #striker tries to get close to ball and shoot if within distance
                dist = math.sqrt(math.pow(xyz_robot1[0]-xyz_ball[0],2) + math.pow(xyz_robot1[1]-xyz_ball[1],2))
                if dist < 0.075:
                    Move_Robot(1000,1000,1)
                    time.sleep(1)
                    Move_Robot(0,0,1)
            else:
                goalkeeper = findgoalkeeper()
                if goalkeeper == 1:
                    robotMoveToThere(1,xyz_bluerobot1[0],xyz_bluerobot1[1]-0.15) #get close to blue team goalkeeper
                if goalkeeper == 2:
                    robotMoveToThere(1,xyz_bluerobot2[0],xyz_bluerobot2[1]-0.15)
                if goalkeeper == 3:
                    robotMoveToThere(1,xyz_bluerobot3[0],xyz_bluerobot3[1]-0.15)
        
        if strategy == 2:
            if flag == 1:
                Move_Robot(1000,1050,1)
                time.sleep(1)
                Move_Robot(0,0,1)
                flag = 2
            if flag == 2:
                robotMoveToThere(1,xyz_ball[0],0)
            while xyz_robot3[1] > -0.6:
                ErrorCode, xyz_robot3 = vrep.simxGetObjectPosition(clientID, Red3_Position, -1, vrep.simx_opmode_buffer)                
                Move_Robot(-750,-810,2)
                Move_Robot(-750,-780,3)
            ErrorCode, xyz_robot1 = vrep.simxGetObjectPosition(clientID, Red1_Position, -1, vrep.simx_opmode_buffer)
            ErrorCode, xyz_ball = vrep.simxGetObjectPosition(clientID, BallPosition, -1, vrep.simx_opmode_buffer)
            dist = math.sqrt(math.pow(xyz_robot1[0]-xyz_ball[0],2) + math.pow(xyz_robot1[1]-xyz_ball[1],2))
            if dist < 0.09 and xyz_ball[1] > 0:
                Move_Robot(1000,1000,1)
                time.sleep(1)
                Move_Robot(0,0,1)
            Move_Robot(-400,400,3)
            Move_Robot(400,-400,2)

vrep.simxFinish(clientID)    
print 'Program ended'