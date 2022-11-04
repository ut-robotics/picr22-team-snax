import math
import numpy as np
import time
import serial
import struct
import os

class OmniMotionRobot:
    def __init__(self):
        #data about robot and motors
        #wheelAngles order = rear, left, right
        self.wheelAngles = [0, 240, 120]
        self.gearboxReductionRatio = 18.75
        self.encoderEdgesPerMotorRevolution = 64
        #perhaps correct this
        self.wheelRadius = 0.035
        self.pidControlFrecuency = 100
        #perhaps correct this
        self.wheelDistanceFromCenter = 0.15
        #should get right port
        self.robotSerialDevice = os.popen("python3.9 -m serial.tools.list_ports").read().split()[0]
        #calculations for sending movement data
        self.wheelSpeedToMainboardUnits = self.gearboxReductionRatio * self.encoderEdgesPerMotorRevolution / (2*3.141528 * self.wheelRadius * self.pidControlFrecuency)

    #sends struct data to mainboard
    def serialCommunication(self, rearSpeed, leftSpeed, rightSpeed, throwerSpeed):
        ser = serial.Serial(self.robotSerialDevice, 115200)
        movementCommand = struct.pack('<hhhHBH', rearSpeed, leftSpeed, rightSpeed, throwerSpeed, True, 0xAAAA)
        ser.write(movementCommand)

    #given in m/s. xSpeed is sideways, ySpeed is forward, rotSpeed in counterclockwise
    def move(self, xSpeed, ySpeed, rotSpeed):
        ySpeed = -ySpeed
        #degrees are counted from the right counterclockwise
        #arctangent
        robotDirection = np.degrees(math.atan2(ySpeed, xSpeed))
        #pythagorean theorem
        robotSpeed = math.sqrt((xSpeed ** 2) + (ySpeed ** 2))
        wheelLinearVelocities = [0,0,0]

        # TODO: do this when implementing turning, not sure if correct
        robotAngularVelocity = rotSpeed

        #calculate wheelLinearVelocities
        for i in range(3):
            wheelLinearVelocities[i] = robotSpeed * math.cos(math.radians(robotDirection - self.wheelAngles[i])) + self.wheelDistanceFromCenter * robotAngularVelocity

        wheelAngularSpeedInMainboardUnits = [0, 0, 0]

        #calculate speeds to send over serial
        for i in range(3):
            wheelAngularSpeedInMainboardUnits[i] = wheelLinearVelocities[i] * self.wheelSpeedToMainboardUnits

        self.serialCommunication(int(wheelAngularSpeedInMainboardUnits[0]), int(wheelAngularSpeedInMainboardUnits[1]), int(wheelAngularSpeedInMainboardUnits[2]), 0)

    # TODO: test this
    #orbit movement around point to find basket
    #positive orbit speed is orbiting orbiting counterclockwise if viewed from above
    def orbit(self, orbitSpeed, orbitRadius):
        orbitCircumference = orbitRadius * 2 * 3.141528
        timeToCompleteFullCircle = orbitCircumference / orbitSpeed
        #in radians
        rotationalSpeed = 2 * 3.141528 / timeToCompleteFullCircle
        #move along x-axis and rotate
        self.move(orbitSpeed, 0, rotationalSpeed)
    
    # TODO: write special movement function to use when approaching ball that uses only rear wheel to adjust direction 

    def throw(self):
        pass


    #stop movement
    def stop(self):
        self.move(0, 0, 0)


    #test function to see if wired correctly
    def testMotors(self):
        self.move(0, 0, 1)
        time.sleep(3)
        self.move(0, 0, 0)

if __name__ == '__main__':
    omniRobot = OmniMotionRobot()
    #omniRobot.serialCommunication(0, 0, 0, 0)
    omniRobot.orbit(0.3, 1)