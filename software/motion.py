import math
import numpy as np
import time
import serial
import struct

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, xSpeed, ySpeed, rotSpeed):
        pass

class OmniMotionRobot(IRobotMotion):
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

        self.robotSerialDevice =

        #calculations for sending movement data
        self.wheelSpeedToMainboardUnits = self.gearboxReductionRatio * self.encoderEdgesPerMotorRevolution / (2*3.141528 * self.wheelRadius * self.pidControlFrecuency)

    #sends struct data to mainboard
    def serialCommunication(self, rearSpeed, leftSpeed, rightSpeed, throwerSpeed):

        ser = serial.Serial(self.robotSerialDevice, 115200)
        movementCommand = struct.pack('<hhhHBH', leftSpeed, rearSpeed, rightSpeed, throwerSpeed, True, 0xAAAA)
        ser.write(movementCommand)

    #given in m/s. xSpeed is sideways, ySpeed is forward, rotSpeed in counterclockwise
    def move(self, xSpeed, ySpeed, rotSpeed):
        speeds = [xSpeed, ySpeed, rotSpeed]
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
    def orbit(self, orbitSpeed, orbitRadius):
        orbitCircumference = orbitRadius * 2 * 3.141528
        timeToCompleteFullCircle = orbitCircumference / orbitSpeed
        #in radians
        rotationalSpeed = 2 * 3.141528 / timeToCompleteFullCircle
        #we want to rotate looking inward
        rotationalSpeed = -rotationalSpeed
        #move along x-axis and rotate
        move(orbitSpeed, 0, rotationalSpeed)

    #stop movement
    def stop():
        move(0, 0, 0)

    #test function to see if wired correctly
    def testMotors(self):
        move(0, 0, 1)
        time.sleep(3)
        move(0, 0, 0)

    def task1(self):
        #robot moves at least 1 metre on the court
        self.move(0.3, 0, 0)
        time.sleep(5)
        self.move(0, 0, 0)

    def task2():
        #find ball, ball is in the middle of camera view and robot not moving
        pass

print(serial.tools.list_ports)
