import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

from controller import Supervisor

from brick_parser import parse_brick_file, to_world_coords
from brick_placer import BrickPool
from shared.vector3 import Vector3
from shared.color import Color
from shared.debug_draw import DebugDraw

BRICK_FILE = "instructions/uni1.txt"

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
dbg = DebugDraw(supervisor, '__DEBUG_BRICK_SPAWNER__')

pool = BrickPool(supervisor)
raw_bricks = parse_brick_file(BRICK_FILE)
bricks = to_world_coords(raw_bricks)

# Calculate Bounds
positions = [pos for pos, _ in bricks]
bounds_min = positions[0]
bounds_max = positions[0]
for pos in positions[1:]:
    bounds_min = Vector3.min(bounds_min, pos)
    bounds_max = Vector3.max(bounds_max, pos)

bounds = bounds_max - bounds_min
boundsCenter = (bounds_min + bounds_max) * 0.5

pool.pre_spawn(len(bricks), physics=False)

SPAWN_INTERVAL = 200 
brick_index = 0
elapsed = 0
state = 'SPAWN'

while supervisor.step(timestep) != -1:
    dbg.clear()
    dbg.draw_wire_box(boundsCenter, bounds, Color.blue)

    elapsed += timestep
    if elapsed >= SPAWN_INTERVAL:
        elapsed -= SPAWN_INTERVAL
        
        if state == 'SPAWN' and brick_index < len(bricks):
            pos, theta = bricks[brick_index]
            pool.spawn(brick_index, pos, theta)
            brick_index += 1
            if brick_index >= len(bricks):
                state = 'DESPAWN'
                brick_index = len(bricks) - 1
        elif state == 'DESPAWN' and brick_index >= 0:
            pool.despawn(brick_index)
            brick_index -= 1

            if brick_index < 0:
                state = 'NONE'
