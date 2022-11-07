from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
from types import *
import image_processor
import camera
import motion
import cv2
import numpy as np
import time
from enum import Enum

# TODO if too much black around ball, ignore it
# TODO create config file for constants
# TODO slightly erode then dilate green pixels
# TODO use depth when throwing
# TODO test if robot.orbit works with neg values


class State(Enum):
    FIND_BALL = 1
    GO_TO_BALL = 2
    ORBIT = 3
    THROW = 4
    MANUAL = 5

class StateMachine:
    def __init__(self, omniRobot, throwIntoBlue = True):
        self.currentState = State.FIND_BALL
        self.robot = omniRobot
        self.imageData = 0
        self.imageWidth = 0
        self.imageHeight = 0
        self.throwIntoBlue = throwIntoBlue
        #self.lastXSpeed = 0
        #self.lastYSpeed = 0
        #self.lastXCoord = 0

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

    #find any ball, slow rotation periodically
    def findBall(self):
        rotationSpeedAdjuster = 0
        if len(self.imageData.balls) == 0:
            if findBallCounter < 30:
                self.robot.move(0,0,1)
                rotationSpeedAdjuster += 1
            elif rotationSpeedAdjuster < 50:
                self.robot.move(0,0,0.5)
                rotationSpeedAdjuster += 1
            else:
                rotationSpeedAdjuster = 0
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
        if normalizedYDistanceFromBottom < 0.25 and abs(normalizedXDistanceFromCenter) < 0.1:
            self.robot.stop()
            self.currentState = State.ORBIT
        
        # TODO calibrate these?
        xSpeedMultiplier = 0.4
        ySpeedMultiplier = 1
        rotSpeedMultiplier = -3

        robotXSpeed = normalizedXDistanceFromCenter * xSpeedMultiplier
        robotYSpeed = (normalizedYDistanceFromBottom - 0.1) * ySpeedMultiplier 
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
        ballCoords = self.getLargestBallCoords(self.imageData)
        ballXCoord = ballCoords[0]
        ballYCoord = ballCoords
        normalizedXDistanceFromCenter = (ballXCoord - (self.imageWidth / 2)) / self.imageWidth
        
        baselineTrajectorySpeed = 0.02
        #if radius is negative, should just orbit the other way around
        baselineRadius = 0.2
        #if basket is centered enough
        basketMaxDistanceFromCenter = 20 #in pixels

        if self.throwIntoBlue:
            if abs(self.imageData.basket_b.x - self.imageWidth / 2) < basketMaxDistanceFromCenter:
                self.currentState = State.THROW
                self.robot.stop()
                return
        else:
            if abs(self.imageData.basket_m.x - self.imageWidth / 2) < basketMaxDistanceFromCenter:
                self.currentState = State.THROW
                self.robot.stop()
                return
        
        #calculate speeds to orbit accurately
        # TODO calibrate
        trajectorySpeedMultiplier = 0.1
        #if ball normxdist < 0 slow down
        correctingTrajectorySpeed = abs(normalizedXDistanceFromCenter) * trajectorySpeedMultiplier

        trajectorySpeed = baselineTrajectorySpeed = correctingTrajectorySpeed

        #if i already see a basket on the left side
        #if its on the left side, all speeds are positive
        if len(self.imageData.basket_b == 1) and self.basket_b.x > (self.imageWidth / 2):
            trajectorySpeed = -trajectorySpeed
            baselineRadius = -baselineRadius
        
        self.robot.orbit(trajectorySpeed, baselineRadius)
        #self.robot.orbit(0.02, 0.20)

    #use rear wheel correcting and set correct thrower speed
    #i have to know that this state always starts at the same distance from the ball and that the basket is almost in the center
    def throw(self):
        self.robot.stop()
        depth = self.imageData.depth_frame
        depth = depth[self.imageHeight-60][424]
        self.robot.serialCommunication(0,0,0,1250) 
        print(depth)
        #dist = 1320, spd = 1000
        #1950, 1250

 
# TODO: RUN COLOR CONFIGURATOR
def main():
    debug = True

    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    
    omniRobot = motion.OmniMotionRobot()
    stateMachine = StateMachine(omniRobot)
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
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=useDepthImage)
            #updating state machine data
            stateMachine.setData(processedData)
            useDepthImage = False

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
                    
                    break
                    

    except KeyboardInterrupt:
        print("closing...")

    finally:
        cv2.destroyAllWindows()
        processor.stop()

if __name__ == '__main__':
    main()
