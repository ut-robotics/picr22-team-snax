from enum import Enum

class State(Enum):
    FIND_BALL = 1
    GO_TO_BALL = 2
    ORBIT = 3
    THROW = 4
    WAIT_REFEREE = 6

    MANUAL = 9
    TESTING = 10


class StateMachine:
    def __init__(self, omniRobot):
        self.currentState = State.FIND_BALL
        self.robot = omniRobot
        self.throwIntoBlue = False

        self.imageData = None
        self.imageWidth = 0
        self.imageHeight = 0

        #for FIND_BALL
        self.rotationSpeedAdjuster = 0

        #for GO_TO_BALL
        self.orbitDone = False

        #for THROW
        self.throwerTimer = 0
        

    def setImageData(self, data):
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
        except:
            self.currentState = State.FIND_BALL
            return

        #ball x coord dist from centre [-1, 1]
        normalizedXDistanceFromCenter = (ballXCoord - (self.imageWidth / 2)) / self.imageWidth
        #y is 1 if ball is far away, value [0, 1]
        normalizedYDistanceFromBottom = (self.imageHeight - ballYCoord) / self.imageHeight

        if normalizedYDistanceFromBottom < 0.30 and abs(normalizedXDistanceFromCenter) < 0.1:
            #if only correcting position
            if self.orbitDone == True:
                self.currentState = State.THROW
                return
            self.currentState = State.ORBIT
            return

        # TODO optimize
        speedMultipliers = {'X': 0.4, 'Y': 1, "ROT": -3}

        robotSpeeds = {'X': normalizedXDistanceFromCenter * speedMultipliers['X'], 'Y': normalizedYDistanceFromBottom * speedMultipliers['Y'], "ROT": normalizedXDistanceFromCenter * speedMultipliers['ROT']}

        self.robot.move(robotSpeeds['X'], robotSpeeds['Y'], robotSpeeds["ROT"])

    def orbit(self):
        try:
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
        basketMaxDistanceFromCenter = 30 #in pixels

        if self.throwIntoBlue:
            if abs(self.imageData.basket_b.x - self.imageWidth / 2) < basketMaxDistanceFromCenter:
                #if orbit is inaccurate, correct position once more
                if abs(normalizedXDistanceFromCenter) > 0.15 or normalizedYDistanceFromBottom > 0.4:
                    self.currentState = State.GO_TO_BALL
                    self.orbitDone = True
                    return
                else:
                    self.currentState = State.THROW
                    return
        else:
            if abs(self.imageData.basket_m.x - self.imageWidth / 2) < basketMaxDistanceFromCenter:
                if abs(normalizedXDistanceFromCenter) > 0.15 or normalizedYDistanceFromBottom > 0.4:
                    self.currentState = State.GO_TO_BALL
                    self.orbitDone = True
                    return
                else:
                    self.currentState = State.THROW
                    return

        trajectorySpeed = 0.3
        
        ySpeedMultiplier = 0.8
        ySpeed = -(0.3 - normalizedYDistanceFromBottom) * ySpeedMultiplier
        
        rotBase = 0.3
        rotSpeedMultiplier = -4
        rotSpeed = rotBase + normalizedXDistanceFromCenter * rotSpeedMultiplier
        
        

        #if already see basket
        #chooses which way to orbit
        if self.throwIntoBlue:
            if self.imageData.basket_b.exists:
                trajectorySpeed = 0.2
                #if basket is on right side of screen, move clockwise
                if self.imageData.basket_b.x > (self.imageWidth / 2):
                    trajectorySpeed = -trajectorySpeed
                    rotSpeed = -rotSpeed
        else:
            if self.imageData.basket_m.exists:
                trajectorySpeed = 0.2
                if self.imageData.basket_m.x > (self.imageWidth / 2):
                    trajectorySpeed = -trajectorySpeed
                    rotSpeed = -rotSpeed

        self.robot.move(trajectorySpeed, 0, rotSpeed)

    #averages pixel distances of a 3x3 square from depth_image
    def basketDist(self, x, y):
        distanceSum = 0
        for i in range(-1, 2):
            for j in range(-1, 2):
                distanceSum += self.imageData.depth_frame[y+i][x+j]
        basketDistance = int(distanceSum / 9)
        return basketDistance

    def throw(self):
        #---------MEASUREMENTS-----------
        #dist 960 spd 900
        #dist 1280 spd 1000
        #dist 2100 spd 1200
        #dist 3250 spd 1450
        #---------MEASUREMENTS-----------

        if self.throwIntoBlue == True:
            basketCenterY = self.imageData.basket_b.y
            basketCenterX = self.imageData.basket_b.x
        else:
            basketCenterY = self.imageData.basket_m.y
            basketCenterX = self.imageData.basket_m.x

        basketDistance = self.basketDist(basketCenterX, basketCenterY)
        
        throwerMultiplier = 0.245 #0.247 #0.234 #2.1 #1.95
        throwerSpeed = round(basketDistance*throwerMultiplier + 610) #695 #685 #300 #390

        print("distance:")        
        print(basketDistance)
        
        print("speed:")
        print(throwerSpeed)

        #move blindly forward and throw for 30 frames
        if self.throwerTimer <= 30:
            self.robot.move(0, 0.25, 0, int(throwerSpeed))        
            self.throwerTimer += 1         
            return  

        #go to beginning
        self.throwerTimer = 0
        self.orbitDone = False
        self.currentState = State.FIND_BALL
