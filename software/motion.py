import turtle
import math
import numpy as np
import time
import tkinter as tk

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, x_speed, y_speed, rot_speed):
        pass

class TurtleRobot(IRobotMotion):
    def __init__(self, name="Default turtle robot"):

        window = tk.Tk()
        window.title(name)

        canvas = tk.Canvas(master=window, width=500, height=500)
        canvas.pack()

        self.screen = turtle.TurtleScreen(canvas)
        self.turtle_obj = turtle.RawTurtle(self.screen)
        self.turtle_obj.speed('fastest')

        self.steps = 20

    def open(self):
        print("Wroom! Starting up turtle!")

    def close(self):
        print("Going to dissapear...")

    #Very dumb logic to draw motion using turtle
    def move(self, x_speed, y_speed, rot_speed):
        self.screen.tracer(0, 0)
        angle_deg = 0

        angle_deg = np.degrees(math.atan2(x_speed, y_speed))

        distance = math.sqrt(math.pow(x_speed, 2) + math.pow(y_speed, 2))

        distance_step = distance / float(self.steps)
        angel_step = np.degrees(rot_speed / float(self.steps))

        self.turtle_obj.penup()
        self.turtle_obj.reset()
        self.turtle_obj.right(angle_deg - 90)
        self.turtle_obj.pendown()

        for i in range(0, self.steps):
            self.turtle_obj.right(angel_step)
            self.turtle_obj.forward(distance_step)

        self.turtle_obj.penup()
        self.screen.update()


class TurtleOmniRobot(TurtleRobot):
    def __init__(self, name="Default turtle omni robot"):
        TurtleRobot.__init__(self, name)

        # Wheel angles
        self.motor_config = [30, 150, 270]

    def move(self, x_speed, y_speed, rot_speed):
        speeds = [0, 0, 0]

        # This is where you need to calculate the speeds for robot motors

        simulated_speeds = self.speeds_to_direction(speeds)

        TurtleRobot.move(self, simulated_speeds[0], simulated_speeds[1], simulated_speeds[2])

    def speeds_to_direction(self, speeds):
        offset_x = 0
        offset_y = 0
        degree = int((speeds[0] + speeds[1] + speeds[2]) / 3)
    
        for i in range(0, 3):
            end_vector = self.motor_side_forward_scale(self.motor_config[i] + 90, speeds[i], offset_x, offset_y)
            offset_x = end_vector[0]
            offset_y = end_vector[1]
    
        offsets = [offset_x * -1, offset_y]
        speeds = [int(a / 1.5) for a in offsets]
        speeds.append(degree)
    
        return speeds
 
    def motor_side_forward_scale(self, angel, length, offset_x=0, offset_y=0):
        ang_rad = math.radians(angel)
        return [length * math.cos(ang_rad) + offset_x, length * math.sin(ang_rad) + offset_y]


class OmniMotionRobot(IRobotMotion):
    def __init__(self, name="Default turtle robot"):

        window = tk.Tk()
        window.title(name)

        canvas = tk.Canvas(master=window, width=500, height=500)
        canvas.pack()

        self.screen = turtle.TurtleScreen(canvas)
        self.turtle_obj = turtle.RawTurtle(self.screen)
        self.turtle_obj.speed('fastest')

        self.steps = 20

    def open(self):
        print("Wroom! Starting up turtle!")

    def close(self):
        print("Going to dissapear...")

    #Very dumb logic to draw motion using turtle
    def move(self, x_speed, y_speed, rot_speed):
        self.screen.tracer(0, 0)
        angle_deg = 0

        angle_deg = np.degrees(math.atan2(x_speed, y_speed))

        distance = math.sqrt(math.pow(x_speed, 2) + math.pow(y_speed, 2))

        distance_step = distance / float(self.steps)
        angel_step = np.degrees(rot_speed / float(self.steps))

        self.turtle_obj.penup()
        self.turtle_obj.reset()
        self.turtle_obj.right(angle_deg - 90)
        self.turtle_obj.pendown()

        for i in range(0, self.steps):
            self.turtle_obj.right(angel_step)
            self.turtle_obj.forward(distance_step)

        self.turtle_obj.penup()
        self.screen.update()