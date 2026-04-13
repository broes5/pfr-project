from controller import Supervisor
import math

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())
timestepSeconds = timestep / 1000.0

node = robot.getFromDef("MY_ROBOT")
translation_field = node.getField("translation")

time = 0.0

def get_f(t):
    return (math.sin(t) + 1) * 0.5;

def lerp(a, b, t):
    return a + ((b -a ) * t)

def get_position(t):
    return [lerp(-0.1, 0.1, get_f((t+400)*0.35)), lerp(-0.1, 0.1, get_f((t+342)*0.25)), lerp(0.2, 0.35, get_f(t*1.2))]


while robot.step(timestep) != -1:
    translation_field.setSFVec3f(get_position(time))
    time += timestepSeconds