from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
from types import *
import image_processor
import camera
import motion
import cv2
import numpy as np
import time
from enum import Enum
import referee

# TODO if too much black around ball, ignore it
# TODO create config file for constants
# TODO slightly erode then dilate green pixels
# TODO use depth when throwing
# TODO test if robot.orbit works with neg values
#dist 1950 spd 1250
#dist 1350 spd 1000


class State(Enum):
    FIND_BALL = 1
    GO_TO_BALL = 2
    ORBIT = 3
    THROW = 4
    MANUAL = 5
    TESTING = 6
    WAIT_REFEREE = 7

class StateMachine:
    def __init__(self, omniRobot):
        self.currentState = State.FIND_BALL
        self.robot = omniRobot
        self.imageData = 0
        self.imageWidth = 0
        self.imageHeight = 0
        self.throwIntoBlue = False
        self.orbitDone = False
        self.rotationSpeedAdjuster = 0
        #self.lastXSpeed = 0
        #self.lastYSpeed = 0
        #self.lastXCoord = 0
        self.throwerTimer = 0
        self.lastDistance = 0
    def setData(self, data):
        self.imageData = data
        
    def setState(self, state):
        self.currentState = state
        if self.currentState == State.FIND_BALL:
            self.findBall()
        if self.currentState == State.GO_TO_BALL:
            self.goToBall()
        if self.currentState == State.ORBIT:
            self.orbit()
        if self.currentState == State.THROW:
            self.throw()
        if self.currentState == State.MANUAL:
            pass
        if self.currentState == State.WAIT_REFEREE:
            self.robot.stop()

    #find any ball, slow rotation periodically
    def findBall(self):
        if len(self.imageData.balls) == 0:
            if self.rotationSpeedAdjuster < 30:
                self.robot.move(0,0,1.5)
                self.rotationSpeedAdjuster += 1
            elif self.rotationSpeedAdjuster < 50:
                self.robot.move(0,0,0.5)
                self.rotationSpeedAdjuster += 1
            else:
                self.rotationSpeedAdjuster = 0
        else:
            self.currentState = State.GO_TO_BALL
    def goToBall(self):
        # TODO cap positive acceleration
        # TODO dont go after balls not in the field
        # TODO if detect black + white line and if ball y is smaller then do 180degrees turn?
        try:
            ballCoords = self.getLargestBallCoords(self.imageData)
            ballXCoord = ballCoords[0]
            ballYCoord = ballCoords[1]
            if len(self.imageData.balls) == 0:
                self.currentState = State.FIND_BALL
                return
        except:
            self.currentState = State.FIND_BALL
            return
        #ball x coord dist from centre [-1, 1], can try different speeds
        normalizedXDistanceFromCenter = (ballXCoord - (self.imageWidth / 2)) / self.imageWidth
        #y is 1 if ball is far away, value [0, 1]
        normalizedYDistanceFromBottom = (self.imageHeight - ballYCoord) / self.imageHeight
        #adjust Y cap 
        if normalizedYDistanceFromBottom < 0.30 and abs(normalizedXDistanceFromCenter) < 0.1:
            self.robot.stop()
            if self.orbitDone == True:
                self.currentState = State.THROW
                return
            self.currentState = State.ORBIT
            return

        # TODO calibrate these?
        xSpeedMultiplier = 0.4
        ySpeedMultiplier = 1
        rotSpeedMultiplier = -3
        robotXSpeed = normalizedXDistanceFromCenter * xSpeedMultiplier
        robotYSpeed = (normalizedYDistanceFromBottom - 0.2) * ySpeedMultiplier 
        robotRotSpeed = normalizedXDistanceFromCenter * rotSpeedMultiplier
        self.robot.move(robotXSpeed, robotYSpeed, robotRotSpeed)
    def getLargestBallCoords(self, imageData):
        largestBallX = 0
        largestBallY = 0
        largestBallSize = 0
        for ball in imageData.balls: 
            if ball.size > largestBallSize:
                largestBallX = ball.x
                largestBallY = ball.y
                largestBallSize = ball.size
        return (largestBallX, largestBallY) 
    
    #currently uses hardcoded distance from ball, try to make go_to as accurate as possible
    # TODO make orbiting dependent on ball distance, possibly just rotate in place to center ball in FOV
    def orbit(self):
        try:
            '''
            if self.getLargestBallCoords[1] < 400:
                self.currentState = State.FIND_BALL
                return
            '''
            ballCoords = self.getLargestBallCoords(self.imageData)
            ballXCoord = ballCoords[0]
            ballYCoord = ballCoords[1]
        except:
            self.currentState = State.FIND_BALL
            return
        normalizedYDistanceFromBottom = (self.imageHeight - ballYCoord) / self.imageHeight
        if normalizedYDistanceFromBottom > 0.6:
            self.currentState = State.FIND_BALL
            return
        normalizedXDistanceFromCenter = (ballXCoord - (self.imageWidth / 2)) / self.imageWidth
        basketMaxDistanceFromCenter = 20 #in pixels
        if self.throwIntoBlue:
            if abs(self.imageData.basket_b.x - self.imageWidth / 2) < basketMaxDistanceFromCenter:
                self.robot.stop()
                if abs(normalizedXDistanceFromCenter) > 0.15 or normalizedYDistanceFromBottom > 0.4:
                    self.currentState = State.GO_TO_BALL
                    self.orbitDone = True
                    return
                else:
                    self.currentState = State.THROW
                    self.robot.stop()
                    return
        else:
            if abs(self.imageData.basket_m.x - self.imageWidth / 2) < basketMaxDistanceFromCenter:
                self.robot.stop()
                if abs(normalizedXDistanceFromCenter) > 0.15 or normalizedYDistanceFromBottom > 0.4:
                    self.currentState = State.GO_TO_BALL
                    return
                else:
                    self.currentState = State.THROW
                    self.robot.stop()
                    return
        trajectorySpeed = 0.3
        rotSpeedMultiplier = -4
        ySpeedMultiplier = 0.8
        
        ySpeed = -(0.3 - normalizedYDistanceFromBottom) * ySpeedMultiplier
        rotSpeed = normalizedXDistanceFromCenter * rotSpeedMultiplier
        rotBase = 0.3

        #if already see basket
        if self.throwIntoBlue:
            if self.imageData.basket_b.exists:
                trajectorySpeed = 0.2
                if self.imageData.basket_b.x > (self.imageWidth / 2):
                    trajectorySpeed = -trajectorySpeed
                    rotSpeed = -rotSpeed
                    rotBase = -rotBase
        else:
            if self.imageData.basket_m.exists:
                trajectorySpeed = 0.2
                if self.imageData.basket_m.x > (self.imageWidth / 2):
                    trajectorySpeed = -trajectorySpeed
                    rotSpeed = -rotSpeed
                    rotBase = -rotBase

        self.robot.move(trajectorySpeed, 0 , rotBase + rotSpeed)
        return
        '''
        baselineTrajectorySpeed = 0.2
        #if radius is negative, should just orbit the other way around
        baselineRadius = 0.25
        #if basket is centered enough
        self.robot.orbit(baselineTrajectorySpeed, baselineRadius)
        '''
    #use rear wheel correcting and set correct thrower speed
    #i have to know that this state always starts at the same distance from the ball and that the basket is almost in the center
    def basketDist(self, x, y):
        sum = 0
        i = -1
        divider = 0
        while i < 2:
            j= -1
            while j < 2:
                divider += 1
                sum += self.imageData.depth_frame[y+i][x+j]
                j += 1
            i += 1
        answer = round(sum / divider)
        return answer

    def throw(self):
        #dist 960 spd 900
        #dist 1280 spd 1000
        #dist 2100 spd 1200
        #dist 3250 spd 1450
        #basketDistance = self.imageData.depth_frame[10][420] #420/10
        if self.throwIntoBlue == True:
            basketCenterY = self.imageData.basket_b.y
            basketCenterX = self.imageData.basket_b.x
        if self.throwIntoBlue == False:
            basketCenterY = self.imageData.basket_m.y
            basketCenterX = self.imageData.basket_m.x

        basketDistance = self.basketDist(basketCenterX, basketCenterY)
        
        throwerMultiplier = 0.245#0.247 #0.234 #2.1 #1.95
        throwerSpeed = round(basketDistance*throwerMultiplier +610) #695 #685 #300 #390
        print("distance:")        
        print(basketDistance)
        
        print("speed:")
        print(throwerSpeed)

        if self.throwerTimer >= 30:
            self.throwerTimer = 0
            self.currentState = State.FIND_BALL
            self.orbitDone = False
            self.lastDistance = 0
            return
        if throwerSpeed < 0:
            throwerSpeed = 1250
        '''
        if self.lastDistance == 0:
            basketDistance = 0

        if self.throwIntoBlue:
            if basketDistance != -1:
                basketDistance = self.imageData.basket_b.distance
                self.lastDistance = basketDistance 
        else:
            if basketDistance != -1:
                basketDistance = self.imageData.basket_m.distance
                self.lastDistance = basketDistance 


        try:
            ballCoords = self.getLargestBallCoords(self.imageData)
            ballXCoord = ballCoords[0]
            ballYCoord = ballCoords[1]
        except:
            ballXCoord = 0
            ballYCoord = 0
        #if ball is close
        if ballYCoord > 300:
            normalizedXDistanceFromCenter = (ballXCoord - (self.imageWidth / 2)) / self.imageWidth
            rotSpeedMultiplier = 1
            rotSpeed = normalizedXDistanceFromCenter * rotSpeedMultiplier
            self.robot.move(0, 0.1, rotSpeed, throwerMultiplier * basketDistance)
        
        '''    
        #self.robot.move(0, 0.25, 0, int(throwerSpeed))
        self.robot.move(0, 0.25, 0, int(throwerSpeed))
        
        #self.robot.move(0, 0.1, 0, int(throwerMultiplier * basketDistance))
        self.throwerTimer += 1           
 
 
# TODO: RUN COLOR CONFIGURATOR
def main():
    competition = False

    debug = True
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    
    omniRobot = motion.OmniMotionRobot()
    stateMachine = StateMachine(omniRobot)
    if competition:
        robotReferee = referee.Referee(ip="192.168.3.220")
        robotReferee.startReferee()
        stateMachine.setState(State.WAIT_REFEREE)

    stateMachine.imageWidth = cam.rgb_width
    stateMachine.imageHeight = cam.rgb_height
    start = time.time()
    fps = 0
    frame = 0
    useDepthImage = False
    #frame counter
    frame_cnt = 0
    try:
        while True:
            if competition:
                cmd = robotReferee.getCommand()
                if (cmd != None):
                    print("COMMAND IN")
                    print(cmd)
                    if cmd[0] == 'START':
                        stateMachine.currentState = State.FIND_BALL 
                        if cmd[1] == 'blue':
                            stateMachine.throwIntoBlue = True
                        else:
                            stateMachine.throwIntoBlue = False

                    elif cmd[0] == 'STOP':
                        stateMachine.currentState = State.WAIT_REFEREE
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=useDepthImage)
            #updating state machine data
            stateMachine.setData(processedData)
            useDepthImage = False

            #stateMachine.setState(State.TESTING)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects
            if stateMachine.currentState == State.FIND_BALL:
                stateMachine.setState(State.FIND_BALL)
            if stateMachine.currentState == State.GO_TO_BALL:
                stateMachine.setState(State.GO_TO_BALL) 
            if stateMachine.currentState == State.ORBIT:
                stateMachine.setState(State.ORBIT)
            if stateMachine.currentState == State.THROW:
                useDepthImage = True
                stateMachine.setState(State.THROW)
            if stateMachine.currentState == State.TESTING:
                stateMachine.setState(State.THROW)
            if stateMachine.currentState == State.WAIT_REFEREE:
                stateMachine.setState(State.WAIT_REFEREE)
            


            frame_cnt +=1
            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                print(stateMachine.currentState)
                try:
                    print(processedData.balls[0].size)
                    print("x: ",processedData.balls[0].x)
                    print("y: ",processedData.balls[0].y)
                except:
                    pass


            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    omniRobot.stop()
                    break
                    

    except KeyboardInterrupt:
        print("closing...")
        robotReferee.disconnect()

    finally:
        cv2.destroyAllWindows()
        processor.stop()


if __name__ == '__main__':
    main()
