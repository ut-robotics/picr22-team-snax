from enum import Enum
import math

class State(Enum):
    FIND_BALL = 1
    GO_TO_BALL = 2
    ORBIT = 3
    THROW = 4
    TESTING = 5
    WAIT_REFEREE = 6
    GO_TO_BASKET = 7

class StateMachine:
    def __init__(self, omniRobot):
        self.currentState = State.FIND_BALL
        self.robot = omniRobot
        self.throwIntoBlue = False

        self.imageData = None
        self.imageWidth = 848
        self.imageHeight = 480
        self.imageCenterPoint = 400

        #for GO_TO_BASKET
        self.patrolCounter = 0

        #for FIND_BALL
        self.rotationSpeedAdjuster = 0

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
        if self.currentState == State.GO_TO_BASKET:
            self.goToBasket()
        if self.currentState == State.ORBIT:
            self.orbit()
        if self.currentState == State.THROW:
            self.throw()
        if self.currentState == State.TESTING:
            self.testing()
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
        self.patrolCounter += 1
        if len(self.imageData.balls) == 0:
            if self.rotationSpeedAdjuster < 30:
                self.robot.move(0,0,1.5)
                self.rotationSpeedAdjuster += 1
            elif self.rotationSpeedAdjuster < 50:
                self.robot.move(0,0,0.5)
                self.rotationSpeedAdjuster += 1
            else:
                self.rotationSpeedAdjuster = 0
            if self.patrolCounter >= 200:
                self.currentState = State.GO_TO_BASKET
                self.patrolCounter = 0
                return
        else:
            self.patrolCounter = 0
            self.currentState = State.GO_TO_BALL
    
    def goToBasket(self):
        if self.throwIntoBlue == True:
            if self.imageData.basket_b.exists:
                if self.patrolCounter < 100:
                    self.robot.move(0,0.7,0)
                    self.patrolCounter += 1
                else:
                    self.patrolCounter = 0
                    self.currentState = State.FIND_BALL
                    return
            else:
                self.robot.move(0,0,0.8)

        else:
            if self.imageData.basket_m.exists:
                if self.patrolCounter < 100:
                    self.robot.move(0,0.7,0)
                    self.patrolCounter += 1
                else:
                    self.patrolCounter = 0
                    self.currentState = State.FIND_BALL
                    return
            else:
                self.robot.move(0,0,0.8)

    def goToBall(self):
        try:
            if len(self.imageData.balls) == 0:
                raise Exception
            ballCoords = self.getLargestBallCoords(self.imageData)
            ballXCoord = ballCoords[0]
            ballYCoord = ballCoords[1]
        except:
            self.currentState = State.FIND_BALL
            return

        #ball x coord dist from centre [-1, 1]
        normalizedXDistanceFromCenter = (ballXCoord - self.imageCenterPoint) / self.imageWidth
        #y is 1 if ball is far away, value [0, 1]
        normalizedYDistanceFromBottom = (self.imageHeight - ballYCoord) / self.imageHeight

        changeToOrbitBallLoc = 0.1
        changeToOrbitBallDist = 0.2

        if normalizedYDistanceFromBottom < changeToOrbitBallDist and abs(normalizedXDistanceFromCenter) < changeToOrbitBallLoc:
            self.currentState = State.ORBIT
            return

        speedMultipliers = {'X': 0.7, 'Y': 1, "ROT": -5}

        robotSpeeds = {'X': normalizedXDistanceFromCenter * speedMultipliers['X'], 'Y': normalizedYDistanceFromBottom * speedMultipliers['Y'], "ROT": normalizedXDistanceFromCenter * speedMultipliers['ROT']}

        self.robot.move(robotSpeeds['X'], robotSpeeds['Y'], robotSpeeds["ROT"])

    def orbit(self):
        try:
            if len(self.imageData.balls) == 0:
                raise Exception
            ballCoords = self.getLargestBallCoords(self.imageData)
            ballXCoord = ballCoords[0]
            ballYCoord = ballCoords[1]
        except:
            self.currentState = State.FIND_BALL
            return

        normalizedXDistanceFromCenter = (ballXCoord - (self.imageCenterPoint)) / self.imageWidth
        normalizedYDistanceFromBottom = (self.imageHeight - ballYCoord) / self.imageHeight

        orbitMaxBallDist = 0.25
        
        #stop if have orbited too far
        if normalizedYDistanceFromBottom > orbitMaxBallDist:
            self.currentState = State.FIND_BALL
            return

        basketMaxDistanceFromCenter = 10 #in pixels

        #check if go into throwing state
        if self.throwIntoBlue:
            if abs(self.imageData.basket_b.x - self.imageCenterPoint) < basketMaxDistanceFromCenter:
                self.currentState = State.THROW
                return
        else:
            if abs(self.imageData.basket_m.x - self.imageCenterPoint) < basketMaxDistanceFromCenter:
                self.currentState = State.THROW
                return


        #speed calculations
        trajectorySpeed = 0.3        
        rotBase = 0.8
        rotSpeedMultiplier = -4
        rotSpeed = rotBase + normalizedXDistanceFromCenter * rotSpeedMultiplier

        #changing orbit speed 
        slowOrbitZone = 100
        seeBasketSpeed = 0.15
        closeBasketSpeed = 0.06

        if self.throwIntoBlue:
            #if i see basket, move slower
            if self.imageData.basket_b.exists:
                trajectorySpeed = seeBasketSpeed
                #if basket is close to center, move even slower
                if abs(self.imageData.basket_b.x - self.imageCenterPoint) < slowOrbitZone:
                    rotSpeed -= 0.6
                    trajectorySpeed = closeBasketSpeed
                #if basket is on right side of screen, move clockwise
                if self.imageData.basket_b.x > (self.imageCenterPoint):
                    trajectorySpeed = -trajectorySpeed
                    rotSpeed = -rotSpeed
        else:
            if self.imageData.basket_m.exists:
                trajectorySpeed = seeBasketSpeed
                if abs(self.imageData.basket_m.x - self.imageCenterPoint) < slowOrbitZone:
                    rotSpeed -= 0.6
                    trajectorySpeed = closeBasketSpeed
                if self.imageData.basket_m.x > (self.imageCenterPoint):
                    trajectorySpeed = -trajectorySpeed
                    rotSpeed = -rotSpeed

        self.robot.move(trajectorySpeed, 0, rotSpeed)


    #averages pixel distances of a 5x5 square from depth_image
    def basketDist(self, x, y):
        distanceSum = 0
        for i in range(-2, 3):
            for j in range(-2, 3):
                distanceSum += self.imageData.depth_frame[y+i][x+j]
        basketDistance = int(distanceSum / 25)
        return basketDistance

    def testing(self):
        try:
            if self.throwIntoBlue == True:
                basketCenterY = self.imageData.basket_b.y
                basketCenterX = self.imageData.basket_b.x
            else:
                basketCenterY = self.imageData.basket_m.y
                basketCenterX = self.imageData.basket_m.x

            basketDistance = self.basketDist(basketCenterX, basketCenterY)

        except:
            basketDistance = 0
        print(basketDistance)
        throwerSpeed = 1500
        self.robot.move(0, 0, 0, int(throwerSpeed))


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
        
        throwerMultiplier = 0.228 #0.254 #0.245 #0.247 #0.234 #2.1 #1.95
        vabaliige = 645 #657
        #throwerSpeed = int(math.log(basketDistance)*457 - 2253) #610 #695 #685 #300 #390
        throwerSpeed = int(basketDistance*throwerMultiplier + vabaliige)


        print("distance:")        
        print(basketDistance)
        
        print("speed:")
        print(throwerSpeed)

        self.robot.move(0, 0.25, 0, int(throwerSpeed))
        #move blindly forward and throw for 30 frames
        if self.throwerTimer <= 22:
            self.robot.move(0, 0.25, 0, int(throwerSpeed))        
            self.throwerTimer += 1         
            return  

        #go to beginning
        self.throwerTimer = 0
        self.currentState = State.FIND_BALL
