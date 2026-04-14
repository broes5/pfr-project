from controller import Robot
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

time = 0.0

while robot.step(timestep) != -1:
    time += dt