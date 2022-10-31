from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
from types import *
import image_processor
import camera
import motion
import cv2
import time
from enum import Enum

class State(Enum):
    FIND_BALL = 1
    GO_TO_BALL = 2
    ORBIT = 3
    THROW = 4
    MANUAL = 5

class StateMachine:
    def __init__(self, omniRobot):
        self.currentState = State.FIND_BALL
        self.robot = omniRobot
        self.imageData = 0
        self.imageWidth = 0
        self.imageHeight = 0
        self.lastXSpeed = 0
        self.lastYSpeed = 0

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

    #find any ball
    def findBall(self):
        if len(self.imageData.balls) == 0:
            self.robot.move(0,0,1)
        else:
            #mine teise state'i
            self.robot.stop()
            self.currentState = State.GO_TO_BALL

    def goToBall(self):
        #proportsionaalne liikumine
        #also positive acceleration is capped

        #yle joonte ei l2he
        #if detect black + white line and if ball y is smaller then do 180degrees turn?
        #dunno how to do this, possibly find holes in orange field

        #TODO calibrate this value
        if ball.size > 300:
            robot.stop()
            #self.currentState = State.ORBIT

        '''
        x koordinaadist lahutatakse pool pildi laiusest ja normaliseeritakse palli kaugus keskpunktist [-1, 1]
        y kiirus on pöördv6rdeline y koordinaadiga

        kiiruse graafik
            ____________
          /  
         /
        /
        '''
        try:
            self.ballCoords = getLargestBallCoords(self.imageData)
            self.ballXCoord = self.ballCoords[0]
            self.ballYCoord = self.ballCoords[1]
        except:
            self.currentState = State.FIND_BALL
            return

        #ball x coord dist from centre [-1, 1], can try different speeds
        self.normalizedXDistanceFromCenter = (self.ballXCoord - (self.imageWidth / 2)) / self.imageWidth
        #y is 1 if ball coord 0, value [0, 1]
        self.normalizedYDistanceFromCenter = (self.imageHeight - self.ballYCoord) / self.imageHeight

        self.xSpeedMultiplier = 1
        self.ySpeedMultiplier = 2
        self.rotSpeedMultiplier = 0.3

        #calculate acceleration, also need to calibrate this.....
        self.robotXSpeed = self.normalizedXDistanceFromCenter * self.xSpeedMultiplier
        self.robotYSpeed = self.normalizedYDistanceFromCenter * self.ySpeedMultiplier 
        self.robotRotSpeed = self.normalizedXDistanceFromCenter * self.rotSpeedMultiplier

        self.robot.move(self.robotXSpeed, self.robotYSpeed, self.robotRotSpeed)


    def getLargestBallCoords(self, imageData):
        self.largestBallX = 0
        self.largestBallY = 0
        self.largestBallSize = 0

        for ball in imageData.balls: 
            if ball.size > self.largestBallSize:
                self.largestBallX = ball.x
                self.largestBallY = ball.y

        return (self.largestBallX, self.largestBallY) 
    
    #1st thing center largest ball???
    #if i see the hoop, orbit accordingly, else just orbit 
    def orbit(self):
        pass

    #use rear wheel correcting and set correct thrower speed
    def throw(self):
        pass




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
    #frame counter
    frame_cnt = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            #updating state machine data
            stateMachine.setData(processedData)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects
            if stateMachine.currentState == State.FIND_BALL:
                stateMachine.setState(State.FIND_BALL)
            if stateMachine.currentState == State.GO_TO_BALL:
                stateMachine.setState(State.GO_TO_BALL) 
            if stateMachine.currentState == State.ORBIT:
                stateMachine.setState(State.ORBIT)
            if stateMachine.currentState == State.THROW:
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
