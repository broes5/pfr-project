import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controller import Robot
from shared.vector3 import Vector3
from shared.brick_parser import parse_brick_file, to_world_coords

BRICK_FILE = "../instructions/uni1.txt"

brick_states = to_world_coords(parse_brick_file(BRICK_FILE))
print(f"[CTRL] Loaded {len(brick_states)} brick states")

N_DRONES = 5
DEBUG_TARGETS = [
    Vector3(0.0, -2.0, 2.0),
    Vector3(0.0, -1.0, 2.0),
    Vector3(0.0,  0.0, 2.0),
    Vector3(0.0,  1.0, 2.0),
    Vector3(0.0,  2.0, 2.0),
]

robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice('emitter')

takeoff_sent = False
targets_sent = False

while robot.step(timestep) != -1:
    t = robot.getTime()

    if not takeoff_sent and t >= 2.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            msg = f"{name} TAKEOFF"
            emitter.send(msg.encode('utf-8'))
            print(f"[CTRL] Sent: {msg}")
        takeoff_sent = True

    # Send targets after drones have had time to reach takeoff altitude,
    # so the TAKEOFF handler doesn't overwrite targetPos.x/y.
    if not targets_sent and t >= 6.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            tgt = DEBUG_TARGETS[i]
            msg = f"{name} TARGET {tgt.to_msg()}"
            emitter.send(msg.encode('utf-8'))
            print(f"[CTRL] Sent: {msg}")
        targets_sent = True
