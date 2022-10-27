from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
import image_processor
import camera
import motion
import cv2
import time
from enum import Enum

# TODO: add more nuanced states?
class State(Enum):
    FIND_BALL = 1
    GO_TO_BALL = 2
    ORBIT = 3
    THROW = 4
    MANUAL = 5

# TODO: make logic, improve class
class StateMachine(self, omniRobot):
    def __init__(self):
        self.currentState = State.FIND_BALL
        self.robot = omniRobot
        self.imageData = 0

    def getData(self, depth):
        if not depth:
            self.imageData = processor.process_frame(aligned_depth = False)
        else:
            self.imageData = processor.process_frame(aligned_depth = True)

    def setState(self, state):
        self.currentState = state

        if self.currentState == State.FIND_BALL:
            #is closest ball in FOV
            if self.findBall() == True:
                self.setState(State.GO_TO_BALL)
            else:
                self.setState(State.FIND_BALL)

        elif self.currentState == State.GO_TO_BALL:
            self.goToBall()

        elif self.currentState == State.ORBIT:
            self.orbit()

        elif self.currentState == State.THROW:
            self.throw()

        elif self.currentState == State.MANUAL:
            pass

    #center ball in field of view
    def findBall(self):
        self.ballCount = len(processedData.balls)

        #find any ball
        while self.ballCount == 0:
            self.robot.move(0,0,0.5)
            time.sleep(0.2)
            self.imageData = self.getData(false)
            self.ballCount = len(processedData.balls)

        robot.stop()

        #arbitraryly small size
        self.largestBallSize = 0
        #find largest ball in FOV and turn to it
        while self.largestBallSize < getLargestBall()[1]:
            self.largestBallSize = getLargestBall()[1]
            self.largestBallIndex = getLargestBall()[0]
            self.ballXCoord = self.imageData.balls[self.largestBallIndex].x

            #decide which way to turn

            if abs(x - (cam.rgb_width / 2)) < 30:
                self.robot.stop()
            elif x - (cam.rgb_width / 2) > 0:
                self.robot.move(0, 0, -0.3)
            elif x - (cam.rgb_width / 2) < 0:
                self.robot.move(0, 0, 3)


        #check if it is still the largest ball in FOV
        self.imageData = self.getData(false)
        if abs(self.largestBallSize - self.getLargestBall()[1]) < 100:
             return True
        else:
            return False

    def getLargestBall(self):
        self.largestBallSize = 0
        #arbitrary large amount
        self.largestBallIndex = 20
        for ball in self.imageData.balls:
            if ball.size > self.largestBallSize:
                self.largestBallSize = ball.size
                self.largestBallIndex = self.imageData.balls.index(ball)
        return (self.largestBallIndex, self.largestBallSize)

    def getBallCount(self):
        pass

    def goToBall(self):
        pass

    def orbit(self):
        pass

    def throw(self):
        pass




# TODO: RUN COLOR CONFIGURATOR

def main():
    omniRobot = motion.OmniMotionRobot(motion.IRobotMotion)
    stateMachine = StateMachine(omniRobot)
    debug = True

    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)

    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()

    start = time.time()
    fps = 0
    frame = 0
    #frame counter
    frame_cnt = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

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
