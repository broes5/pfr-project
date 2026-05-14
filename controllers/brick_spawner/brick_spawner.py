import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

from controller import Supervisor

from brick_parser import parse_brick_file, to_world_coords
from brick_placer import BrickPlacer
from vector3 import Vector3

BRICK_FILE = "instructions/uni1.txt"

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

placer = BrickPlacer(supervisor)
raw_bricks = parse_brick_file(BRICK_FILE)

bricks = to_world_coords(raw_bricks)
placer.spawn_many(bricks, physics=False)

# This needs to exist otherwise Webots sometimes gets upset
while supervisor.step(timestep) != -1:
    pass