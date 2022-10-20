from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
import image_processor
import camera
import motion
import cv2
import time

x_old = 0
y_old = 0
side_speed = 0
forward_speed = 0
rot_speed = 0

# TODO: RUN COLOR CONFIGURATOR

def main():
    debug = True

    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)

    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    iRobot = motion.IRobotMotion()
    omniRobot = motion.OmniMotionRobot()
    omniRobot.task1()

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
                    robo_x =processedData.balls[0].x
                    robo_x -= 424
                    robo_y =processedData.balls[0].y
                    robo_y -= 320

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
