#Import Libraries:
import vrep #import library for VREP API
import time #time library
import math

# Determines speed to kick the ball to goal
def kick_speed(robot_position, goal_position):
    dist = distance(robot_position, goal_position)
    kick_speed = (dist/100) * 30 - 2
    #print "kick_speed", kick_speed
    return kick_speed

# Calculates the goal position accordning to the orientation of next player
def calc_goal(r2_position, r2_orientation):
    if r2_orientation[2] < math.pi/2 and r2_orientation[2] > -math.pi/2:
        #looks to the left
        return [r2_position[0]-0.1, r2_position[1] + 0.25]
    else:
        #looks to the right
        return [r2_position[0]-0.1, r2_position[1] - 0.25]

# Compute the desired_angle R1 has to kick in order to get the ball to R2
def calc_desired_angle(r2_position, r2_orientation):
    if r2_orientation[2] < math.pi/2 and r2_orientation[2] > -math.pi/2:
        #looks to the left
        desired_angle = returnHeading(Ball_position, [r2_position[0], r2_position[1] + 0.15])
    else:
        #looks to the right
        desired_angle = returnHeading(Ball_position, [r2_position[0], r2_position[1] - 0.15])
    return desired_angle

# Returns the position the robot has to be to kick the ball to desired_angle
def calc_pos(desired_angle):
    alpha = 0
    if desired_angle > 0:
        alpha = desired_angle - math.pi
    else:
        alpha = desired_angle + math.pi

    dist = 0.1
    dx = -math.sin(alpha) * dist
    dy = math.cos(alpha) * dist
    new_x = Ball_position[0] + dx
    new_y = Ball_position[1] + dy
    #print Ball_position
    #print new_x, new_y
    return [new_x, new_y]

# Moves robot with given speed and given heading
def pid(heading, speed, robot_orientation, leftJointHandle, rightJointHandle, esum):
    P = 0.5
    I = 0.5
    delta = robot_orientation[2] - heading
    if delta < -math.pi:
        delta = delta + 2 * math.pi
    if delta > math.pi:
        delta = delta - 2 * math.pi
    esum = esum + delta
    setSpeed(P*delta+I*delta+speed, -P*delta-I*delta+speed, leftJointHandle, rightJointHandle)
    #print delta
    if delta > 0.02 or delta < -0.02:
        return False
    else:
        return True

def pid2(heading, speed, robot_orientation, leftJointHandle, rightJointHandle, esum):
    P = 1.5
    I = 0.5
    delta = robot_orientation[2] - heading
    if delta < -math.pi:
        delta = delta + 2 * math.pi
    esum = esum + delta
    setSpeed(P*delta+I*delta+speed, -P*delta-I*delta+speed, leftJointHandle, rightJointHandle)
    #print delta
    if delta > 0.02 or delta < -0.02:
        return False
    else:
        return True

# Turns the robot bevore the kick
def kick1(robot_position, robot_orientation, goal_position, leftJointHandle, rightJointHandle, esum):
    heading = returnHeading(robot_position, goal_position)
    if not pid(heading, 0, robot_orientation, leftJointHandle, rightJointHandle, esum):
        #print 'turning'
        return False
    return True

# Moves forward for 0.1 seconds with given speed
def kick2(speed, start, robot_position, robot_orientation, goal_position, leftJointHandle, rightJointHandle, esum):
    #print 'timedelta', time.time(), start, (time.time() - start)
    if time.time() - start < 1:
        heading = returnHeading(robot_position, goal_position)
        pid(heading, speed, robot_orientation, leftJointHandle, rightJointHandle, esum)
        return False
    else:
        setSpeed(0,0, leftJointHandle, rightJointHandle)
        return True

# Set speed of given handle
def setSpeed(leftSpeed, rightSpeed, leftJointHandle, rightJointHandle):
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,leftJointHandle,leftSpeed,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,rightJointHandle,rightSpeed,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
    #ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,leftSpeed,vrep.simx_opmode_oneshot_wait) #set Left wheel speed

# Computes euclidian distance between Pos1 and Pos2
def distance(Pos1, Pos2):
    dx = 100 * (Pos1[0] - Pos2[0])
    dy = 100 * (Pos1[1] - Pos2[1])
    dist = math.sqrt(dx*dx + dy*dy)
    return dist

# Returns desired heading to move to goal position
def returnHeading(robot_position, goal_position):
    dx = math.fabs(100 * (robot_position[0] - goal_position[0]))
    dy = math.fabs(100 * (robot_position[1] - goal_position[1]))
    #print 'dx=', dx, 'dy=', dy
    phi = math.atan(dy/dx)
    #print 'phi=', phi
    des = 0
    if robot_position[0] > goal_position[0]:
        if robot_position[1] > goal_position[1]:
            #print '1'
            des = math.pi/2 + phi
        else:
            #print '2'
            des = math.pi/2 - phi
    else:
        if robot_position[1] > goal_position[1]:
            des = -math.pi/2 - phi
            #print '3'
        else:
            des = -math.pi/2 + phi
            #print '4'

    #print 'des', des
    #print 'robot', robot_position[0], robot_position[1]
    #print 'goal', goal_position[0], goal_position[1]
    return des

# Moves the robot backwards, if it's too close to the ball
def moveBack(robot_position, leftJointHandle, rightJointHandle):
    #print "DISTANCE", distance(Ball_position, robot_position)
    if distance(Ball_position, robot_position) < 20:
        setSpeed(-2, -2, leftJointHandle, rightJointHandle)
        time.sleep(4)
        setSpeed(0, 0, leftJointHandle, rightJointHandle)

# Moves to ball in order to kick it in the desired_angle
def moveToBall(desired_angle, robot_position, robot_orientation, robot_prox1, robot_prox2, speed, leftJointHandle, rightJointHandle, esum):
    goal_position = calc_pos(desired_angle)
    des_heading = returnHeading(robot_position, goal_position)

    if distance(robot_position, goal_position) > 5: # not close enough to goal
        pid(des_heading, 2, robot_orientation, leftJointHandle, rightJointHandle, esum)
        return False

    if distance(robot_position, goal_position) > 2: #very close
        if not pid(des_heading, 0, robot_orientation, leftJointHandle, rightJointHandle, esum):
            return False
        pid(des_heading, 2, robot_orientation, leftJointHandle, rightJointHandle, esum)



        # if robot_prox1:
        #     #turn right
        #     des_heading -= math.pi/4
        #     des_heading = des_heading % math.pi if des_heading < -180 else des_heading
        #     t = time.time()
        #     while (time.time() - t) < 2:
        #         pid(des_heading, 2, robot_orientation, leftJointHandle, rightJointHandle, esum)
        #
        # if robot_prox2:
        #     #turn left
        #     des_heading += math.pi/4
        #     des_heading = des_heading % (-math.pi) if des_heading > 180 else des_heading
        #     t = time.time()
        #     while (time.time() - t) < 2:
        #         pid(des_heading, 2, robot_orientation, leftJointHandle, rightJointHandle, esum)

        return False

    else:
        setSpeed(0, 0, leftJointHandle, rightJointHandle)
        return True

# Moves the robot to goal_position with goal_heading
def moveToPos(robot_position, robot_orientation, goal_position, goal_heading, robot_prox1, robot_prox2, leftJointHandle, rightJointHandle, esum):
    des_heading = returnHeading(robot_position, goal_position)
    dist = distance(robot_position, goal_position)
    speed = dist/8
    #print speed, dist
    if dist > 5: # not close enough to goal
            pid2(des_heading, speed, robot_orientation, leftJointHandle, rightJointHandle, esum)
            return False
    else:
        # if not pid(goal_heading, 0, robot_orientation, leftJointHandle, rightJointHandle, esum):
        #     return False
        if dist < 5:
            if not pid(goal_heading, 0, robot_orientation, leftJointHandle, rightJointHandle, esum):
                return False
            else:
                setSpeed(0, 0, leftJointHandle, rightJointHandle)
                return True

def moveToPos2(robot_position, robot_orientation, goal_position, goal_heading, robot_prox1, robot_prox2, leftJointHandle, rightJointHandle, esum):
    des_heading = returnHeading(robot_position, goal_position)

    if distance(robot_position, goal_position) > 5: # not close enough to goal
            pid(des_heading, 2, robot_orientation, leftJointHandle, rightJointHandle, esum)
            return False
    else:
        if not pid(goal_heading, 0, robot_orientation, leftJointHandle, rightJointHandle, esum):
            return False
        setSpeed(0, 0, leftJointHandle, rightJointHandle)
        return True

# Checks wether the goal keeper is at the bottom of the goal i.e. goal is free
def checkGoal(keeper_position):
    if keeper_position[0] < -0.13:
        return True
    else:
        return False

#Initialisation for Python to connect to VREP
print 'Python program started'
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print 'Connected to V-REP'
else:
    print 'Failed connecting to V-REP'
    vrep.simxFinish(clientID)


if clientID!=-1:

    # Get Ball
    ErrorCode, Ball = vrep.simxGetObjectHandle(clientID, 'Ball',vrep.simx_opmode_oneshot_wait)
    #ErrorCode = vrep.simxSetObjectPosition(clientID, Ball, -1, {-0.211, -0.451, 0.0250}, vrep.simx_opmode_oneshot_wait)
    ErrorCode, Ball_position = vrep.simxGetObjectPosition(clientID, Ball,-1,vrep.simx_opmode_oneshot_wait)

    # Get Goalkeeper
    ErrorCode, Keeper = vrep.simxGetObjectHandle(clientID, 'Red3', vrep.simx_opmode_oneshot_wait)
    ErrorCode, Keeper_position = vrep.simxGetObjectPosition(clientID, Keeper, -1, vrep.simx_opmode_oneshot_wait)

    #Get Blue1
    ErrorCode, Blue1_LeftJointHandle = vrep.simxGetObjectHandle(clientID, 'Blue1_leftJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue1_RightJointHandle = vrep.simxGetObjectHandle(clientID, 'Blue1_rightJoint',vrep.simx_opmode_oneshot_wait)

    ErrorCode, Blue1 = vrep.simxGetObjectHandle(clientID, 'Blue1',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue1_position = vrep.simxGetObjectPosition(clientID, Blue1,-1,vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue1_orientation = vrep.simxGetObjectOrientation(clientID, Blue1,-1,vrep.simx_opmode_oneshot_wait)

    #Get Blue2
    ErrorCode, Blue2_LeftJointHandle = vrep.simxGetObjectHandle(clientID, 'Blue2_leftJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue2_RightJointHandle = vrep.simxGetObjectHandle(clientID, 'Blue2_rightJoint',vrep.simx_opmode_oneshot_wait)

    ErrorCode, Blue2 = vrep.simxGetObjectHandle(clientID, 'Blue2',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue2_position = vrep.simxGetObjectPosition(clientID, Blue2,-1,vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue2_orientation = vrep.simxGetObjectOrientation(clientID, Blue2,-1,vrep.simx_opmode_oneshot_wait)

    #Get Blue3
    ErrorCode, Blue3_LeftJointHandle = vrep.simxGetObjectHandle(clientID, 'Blue3_leftJoint',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue3_RightJointHandle = vrep.simxGetObjectHandle(clientID, 'Blue3_rightJoint',vrep.simx_opmode_oneshot_wait)

    ErrorCode, Blue3 = vrep.simxGetObjectHandle(clientID, 'Blue3',vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue3_position = vrep.simxGetObjectPosition(clientID, Blue3,-1,vrep.simx_opmode_oneshot_wait)
    ErrorCode, Blue3_orientation = vrep.simxGetObjectOrientation(clientID, Blue3,-1,vrep.simx_opmode_oneshot_wait)

    #Get proxSens handles
    ErrorCode, Blue1_ProxSensorHandle1 = vrep.simxGetObjectHandle(clientID,'Blue1_proxSensor2',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, Blue1_ProxSensorHandle2 = vrep.simxGetObjectHandle(clientID,'Blue1_proxSensor3',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, Blue1_state1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(clientID, Blue1_ProxSensorHandle1,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
    ErrorCode, Blue1_state2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, Blue1_ProxSensorHandle2,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run

    ErrorCode, Blue2_ProxSensorHandle1 = vrep.simxGetObjectHandle(clientID,'Blue2_proxSensor2',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, Blue2_ProxSensorHandle2 = vrep.simxGetObjectHandle(clientID,'Blue2_proxSensor3',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, Blue2_state1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(clientID, Blue2_ProxSensorHandle1,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
    ErrorCode, Blue2_state2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, Blue2_ProxSensorHandle2,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run

    ErrorCode, Blue3_ProxSensorHandle1 = vrep.simxGetObjectHandle(clientID,'Blue3_proxSensor2',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, Blue3_ProxSensorHandle2 = vrep.simxGetObjectHandle(clientID,'Blue3_proxSensor3',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, Blue3_state1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(clientID, Blue3_ProxSensorHandle1,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
    ErrorCode, Blue3_state2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, Blue3_ProxSensorHandle2,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run

    #print ErrorCode
    t = time.time()
    runtime = 300
    esum = 0
    start = 0
    speed = 5

    step0 = False
    step1 = False
    step2 = False
    step3 = False
    step4 = False
    step5 = False
    step6 = False
    step7 = False
    step8 = False
    step9 = False
    step10 = False
    step11 = False
    step12 = False
    step13 = False
    step14 = False
    step15 = False
    step16 = False
    step17 = False
    step18 = False
    step19 = False
    step20 = False

    while (time.time()-t)<runtime: #End program after 120sec
        ErrorCode, Ball_position = vrep.simxGetObjectPosition(clientID, Ball,-1,vrep.simx_opmode_oneshot_wait)
        ErrorCode, Keeper_position = vrep.simxGetObjectPosition(clientID, Keeper, -1, vrep.simx_opmode_oneshot_wait)#-0.17

        ErrorCode, Blue1_position = vrep.simxGetObjectPosition(clientID, Blue1,-1,vrep.simx_opmode_oneshot_wait)
        ErrorCode, Blue2_position = vrep.simxGetObjectPosition(clientID, Blue2,-1,vrep.simx_opmode_oneshot_wait)
        ErrorCode, Blue3_position = vrep.simxGetObjectPosition(clientID, Blue3,-1,vrep.simx_opmode_oneshot_wait)
        ErrorCode, Blue1_orientation = vrep.simxGetObjectOrientation(clientID, Blue1,-1,vrep.simx_opmode_oneshot_wait)
        ErrorCode, Blue2_orientation = vrep.simxGetObjectOrientation(clientID, Blue2,-1,vrep.simx_opmode_oneshot_wait)
        ErrorCode, Blue3_orientation = vrep.simxGetObjectOrientation(clientID, Blue3,-1,vrep.simx_opmode_oneshot_wait)

        ErrorCode, Blue1_state1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(clientID, Blue1_ProxSensorHandle1,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
        ErrorCode, Blue1_state2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, Blue1_ProxSensorHandle2,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
        ErrorCode, Blue2_state1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(clientID, Blue2_ProxSensorHandle1,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
        ErrorCode, Blue2_state2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, Blue2_ProxSensorHandle2,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
        ErrorCode, Blue3_state1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(clientID, Blue3_ProxSensorHandle1,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run
        ErrorCode, Blue3_state2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, Blue3_ProxSensorHandle2,vrep.simx_opmode_oneshot_wait) #Get bumper force reading, first run


        remaining = runtime-(time.time()-t)
        #print "Time Remaining=", remaining

        # Move to initial position
        if not step0:
            #posB1 = [-0.4, -0.6]
            posB2 = [0.3, -0.55]
            posB3 = [0.125, 0.5]
            #b1 = moveToPos(Blue1_position, Blue1_orientation, posB1, 0, Blue1_state1, Blue1_state2, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)
            b2 = moveToPos(Blue2_position, Blue2_orientation, posB2, -0.1, Blue2_state1, Blue2_state2, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)
            b3 = moveToPos(Blue3_position, Blue3_orientation, posB3, -2.1, Blue3_state1, Blue3_state2, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)
            step0 = b2 and b3

        # Blue1 moves to ball
        if step0 and not step1:
            desired_angle = calc_desired_angle(Blue2_position, Blue2_orientation)
            step1 = moveToBall(desired_angle, Blue1_position, Blue1_orientation, Blue1_state1, Blue1_state2, 2, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)

        # Blue1 kicks the ball to Blue2
        if step1 and not step2:
            #print "KICK1_1"
            goal = calc_goal(Blue2_position, Blue2_orientation)
            step2 = kick1(Blue1_position, Blue1_orientation, goal, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)
            if step2:
                start = time.time()
                speed = 5
        if step1 and step2 and not step3:
            step3 = kick2(speed, start, Blue1_position, Blue1_orientation, goal, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)
            speed = kick_speed(Blue1_position, goal)
            if step3:
                setSpeed(-2, -2, Blue1_LeftJointHandle, Blue1_RightJointHandle)
                time.sleep(8)
                setSpeed(0, 0, Blue1_LeftJointHandle, Blue1_RightJointHandle)
                time.sleep(4) #kick doen, Blue2 has to wait for the ball
                moveBack(Blue2_position, Blue2_LeftJointHandle, Blue2_RightJointHandle)

        # Blue2 moves to Ball
        if step1 and step2 and step3 and not step4:
            desired_angle = calc_desired_angle(Blue3_position, Blue3_orientation)
            step4 = moveToBall(desired_angle, Blue2_position, Blue2_orientation, Blue2_state1, Blue2_state2, 2, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)

        # Blue2 kicks the ball to Blue3
        if step1 and step2 and step3 and step4 and not step5:
            #print "KICK2_1"
            goal = calc_goal(Blue3_position, Blue3_orientation)
            step5 = kick1(Blue2_position, Blue2_orientation, goal, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)
            if step5:
                start = time.time()
                speed = 5
        if step1 and step2 and step3 and step4 and step5 and not step6:
            step6 = kick2(speed, start, Blue2_position, Blue2_orientation, goal, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)
            speed = kick_speed(Blue2_position, goal)

        # Blue1 moves to new position
        if step1 and step2 and step3 and step4 and step5 and step6 and not step7:
            pos = [-0.2, 0.4]
            step7 = moveToPos(Blue1_position, Blue1_orientation, pos, math.pi, Blue1_state1, Blue1_state2, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)
            if step7:
                moveBack(Blue3_position, Blue3_LeftJointHandle, Blue3_RightJointHandle)

        # Blue3 moves to Ball
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and not step8:
            desired_angle = calc_desired_angle(Blue1_position, Blue1_orientation)
            step8 = moveToBall(desired_angle, Blue3_position, Blue3_orientation, Blue3_state1, Blue3_state2, 2, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)

        # Blue 3 kicks the Ball to Blue1
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and not step9:
            goal = calc_goal(Blue1_position, Blue1_orientation)
            step9 = kick1(Blue3_position, Blue3_orientation, goal, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)
            if step9:
                start = time.time()
                speed = 5
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and not step10:
            step10 = kick2(speed, start, Blue3_position, Blue3_orientation, goal, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)
            speed = kick_speed(Blue3_position, goal)-8
            if step10:
                #move back Blue2/Blue3
                setSpeed(-2, -2, Blue2_LeftJointHandle, Blue2_RightJointHandle)
                setSpeed(-2, -2, Blue3_LeftJointHandle, Blue3_RightJointHandle)
                time.sleep(8)
                setSpeed(0, 0, Blue2_LeftJointHandle, Blue2_RightJointHandle)
                setSpeed(0, 0, Blue3_LeftJointHandle, Blue3_RightJointHandle)

        # Blue2 moves to new position
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and not step11:
            pos = [-0.4, -0.4]
            step11 = moveToPos(Blue2_position, Blue2_orientation, pos, -math.pi/2, Blue2_state1, Blue2_state2, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)
            if step11:
                moveBack(Blue1_position, Blue1_LeftJointHandle, Blue1_RightJointHandle)

        # Blue1 moves to ball
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and not step12:
            desired_angle = calc_desired_angle(Blue2_position, Blue2_orientation)
            step12 = moveToBall(desired_angle, Blue1_position, Blue1_orientation, Blue1_state1, Blue1_state2, 2, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)

        # Blue1 kicks the ball to Blue2
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and not step13:
            goal = calc_goal([-0.1, -0.4], Blue2_orientation)
            step13 = kick1(Blue1_position, Blue1_orientation, goal, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)
            if step13:
                start = time.time()
                speed = 5
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and not step14:
            step14 = kick2(speed, start, Blue1_position, Blue1_orientation, goal, Blue1_LeftJointHandle, Blue1_RightJointHandle, esum)
            speed = kick_speed(Blue1_position, goal)-2.25
            
        # Blue3 moves to new position
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and not step15:
            pos = [0.3, -0.3]
            step15 = moveToPos2(Blue3_position, Blue3_orientation, pos, math.pi/2 + 0.1, Blue3_state1, Blue3_state2, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)
            if step15:
                moveBack(Blue2_position, Blue2_LeftJointHandle, Blue2_RightJointHandle)

        # Blue2 moves to Ball
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and step15 and not step16:
            desired_angle = calc_desired_angle(Blue3_position, Blue3_orientation)
            step16 = moveToBall(desired_angle, Blue2_position, Blue2_orientation, Blue2_state1, Blue2_state2, 2, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)

        # Blue2 kicks the ball to Blue3
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and step15 and step16 and not step17:
            goal = calc_goal(Blue3_position, Blue3_orientation)
            step17 = kick1(Blue2_position, Blue2_orientation, goal, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)
            if step17:
                start = time.time()
                speed = 5
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and step15 and step16 and step17 and not step18:
            step18 = kick2(speed, start, Blue2_position, Blue2_orientation, goal, Blue2_LeftJointHandle, Blue2_RightJointHandle, esum)
            speed = kick_speed(Blue2_position, goal)
            if step18:
                moveBack(Blue3_position, Blue3_LeftJointHandle, Blue3_RightJointHandle)
                time.sleep(8) # wait for the ball

        # Blue3 move to ball
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and step15 and step16 and step17 and step18 and not step19:
            desired_angle = calc_desired_angle([0.0, -0.75], [0, 0, math.pi]) #center of goal
            step19 = moveToBall(desired_angle, Blue3_position, Blue3_orientation, Blue3_state1, Blue3_state2, 2, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)

        # Blue3 kicks the ball to goal
        goal_free = checkGoal(Keeper_position)
        # Blue3 kicks the ball to goal
        if step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and step15 and step16 and step17 and step18 and step19 and not step20:
            goal = calc_goal([0.0, -0.75], [0, 0, math.pi])
            step20 = kick1(Blue3_position, Blue3_orientation, goal, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)
        if goal_free and step1 and step2 and step3 and step4 and step5 and step6 and step7 and step8 and step9 and step10 and step11 and step12 and step13 and step14 and step15 and step16 and step17 and step18 and step19 and step20:
            speed = kick_speed(Blue3_position, goal)
            heading = returnHeading(Blue3_position, goal)
            pid(heading, speed, Blue3_orientation, Blue3_LeftJointHandle, Blue3_RightJointHandle, esum)
            time.sleep(1)
            setSpeed(-20, -20, Blue3_LeftJointHandle, Blue3_RightJointHandle)
            time.sleep(0.5)
            setSpeed(0, 0, Blue3_LeftJointHandle, Blue3_RightJointHandle)
            break #stop program

        time.sleep(0.05) #sleep 50ms

    #stop all motors
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,Blue1_LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,Blue1_RightJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,Blue2_LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,Blue2_RightJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,Blue3_LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,Blue3_RightJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor

vrep.simxFinish(clientID)
print 'Program ended'
