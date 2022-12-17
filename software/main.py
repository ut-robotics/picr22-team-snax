from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
from types import *
import image_processor
import camera
import motion
import statemachine
import cv2
import numpy as np
import time
import referee

# TODO if too much black around ball, ignore it
# TODO create config file for constants
# TODO make robot drive towards other basket if point basket is too close


def main():
    #---------
    #config
    competition = True
    debug = True
    #---------

    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    
    omniRobot = motion.OmniMotionRobot()
    stateMachine = statemachine.StateMachine(omniRobot)
    stateMachine.throwIntoBlue = True
    stateMachine.imageWidth = cam.rgb_width
    stateMachine.imageHeight = cam.rgb_height
    #stateMachine.currentState = statemachine.State.GO_TO_BASKET

    if competition:
        robotReferee = referee.Referee(ip="192.168.3.220", port="8111")
        robotReferee.startReferee()
        stateMachine.setState(statemachine.State.WAIT_REFEREE)

    start = time.time()
    fps = 0
    frame = 0
    useDepthImage = False
    #frame counter
    frame_cnt = 0
    try:
        while True:
            #referee handling
            if competition:
                cmd = robotReferee.getCommand()
                if (cmd != None):
                    if cmd[0] == 'START':
                        #stateMachine.currentState = statemachine.State.GO_TO_BASKET
                        stateMachine.currentState = statemachine.State.FIND_BALL
                        if cmd[1] == 'blue':
                            stateMachine.throwIntoBlue = True
                        else:
                            stateMachine.throwIntoBlue = False

                    elif cmd[0] == 'STOP':
                        stateMachine.currentState = statemachine.State.WAIT_REFEREE
            
            
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=useDepthImage)
            #updating state machine data
            stateMachine.setImageData(processedData)

            useDepthImage = False

            if stateMachine.currentState == statemachine.State.FIND_BALL:
                stateMachine.setState(statemachine.State.FIND_BALL)
            if stateMachine.currentState == statemachine.State.GO_TO_BALL:
                stateMachine.setState(statemachine.State.GO_TO_BALL) 
            if stateMachine.currentState == statemachine.State.GO_TO_BASKET:
                stateMachine.setState(statemachine.State.GO_TO_BASKET)
            if stateMachine.currentState == statemachine.State.ORBIT:
                stateMachine.setState(statemachine.State.ORBIT)
            if stateMachine.currentState == statemachine.State.THROW:
                useDepthImage = True
                stateMachine.setState(statemachine.State.THROW)
            if stateMachine.currentState == statemachine.State.TESTING:
                useDepthImage = True
                stateMachine.setState(statemachine.State.TESTING)
            if stateMachine.currentState == statemachine.State.WAIT_REFEREE:
                stateMachine.setState(statemachine.State.WAIT_REFEREE)

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

    finally:
        if competition:
            robotReferee.disconnect()
        cv2.destroyAllWindows()
        processor.stop()

if __name__ == '__main__':
    main()
