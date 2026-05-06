from controller import Robot
import math

from drone_setup import Drone

robot = Robot()

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

time = 0.0

print("Starting up drone...")
drone = Drone(robot)

while robot.step(timestep) != -1:
    time += dt