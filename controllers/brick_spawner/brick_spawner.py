from controller import Supervisor

from brick_parser import parse_brick_file, to_world_coords
from brick_placer import BrickPlacer

BRICK_FILE = "uni1.txt"

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

placer = BrickPlacer(supervisor)
raw_bricks = parse_brick_file(BRICK_FILE)
positions = to_world_coords(raw_bricks)
placer.spawn_many(positions, physics=False)

while supervisor.step(timestep) != -1:
    pass