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
            #mine teise state'iq
            self.robot.stop()
            self.currentState = State.GO_TO_BALL

    def goToBall(self):
        #proportsionaalne liikumine
        '''
        x koordinaadist lahutatakse pool pildi laiusest ja normaliseeritakse palli kaugus keskpunktist [-1, 1]
        y kiirus on pöördv6rdeline y koordinaadiga

        kiiruse graafik
            ____________
          /  
         /
        /
        '''
        pass

    def orbit(self):
        pass

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

    start = time.time()
    fps = 0
    frame = 0
    #frame counter
    frame_cnt = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            stateMachine.setData(processedData)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects
            
            stateMachine.setState(State.FIND_BALL)

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
