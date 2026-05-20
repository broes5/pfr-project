import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from collections import deque
from controller import Robot
from shared.vector3 import Vector3
from shared.voxel_world import VoxelWorld
from shared.pathfinder import find_path
from shared.brick_parser import parse_brick_file, to_world_coords

BRICK_FILE = "../instructions/uni1.txt"
PILE_POSITION = Vector3(-4.0, 0.0, 0.0)
TASK_ALT = 3.0      # cruise altitude; must match drone's TASK_ALT
TAKEOFF_ALT = 3.0   # used to infer when drones are airborne from CURRENTPOS

N_DRONES = 3

WORLD_ORIGIN = Vector3(-8.0, -8.0, 0.0)
WORLD_SIZE   = Vector3(16.0, 16.0, 8.0)
VOXEL_SIZE   = 0.5

brick_targets = to_world_coords(parse_brick_file(BRICK_FILE))
print(f"[CTRL] Loaded {len(brick_targets)} brick targets")

robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(timestep)

voxel_world = VoxelWorld(WORLD_ORIGIN, WORLD_SIZE, VOXEL_SIZE)

# Brick lifecycle tracking
unplaced  = deque(range(len(brick_targets)))  # brick_ids not yet placed
placed    = set()                              # brick_ids successfully placed
in_flight = {}                                 # drone_name -> brick_id being carried

drone_positions = {}  # drone_name -> last known Vector3

takeoff_sent = False


def _send_path_and_task(drone_name, goal, task_msg):
    """Compute an A* path to goal and send PATH + task_msg to the drone."""
    start = drone_positions.get(drone_name)
    if start is not None:
        path = find_path(voxel_world, start, goal)
        if path and len(path) > 0:
            wp_strs = ' '.join(f"{wp.x},{wp.y},{wp.z}" for wp in path)
            emitter.send(f"{drone_name} PATH {wp_strs}".encode('utf-8'))
            print(f"[CTRL] PATH ({len(path)} wps) → {drone_name} to ({goal.x:.2f},{goal.y:.2f},{goal.z:.2f})")
        else:
            print(f"[CTRL] No A* path found for {drone_name}: start=({start.x:.2f},{start.y:.2f},{start.z:.2f}) goal=({goal.x:.2f},{goal.y:.2f},{goal.z:.2f}), task will use fallback")
    emitter.send(task_msg.encode('utf-8'))


def assign_brick(drone_name):
    """Assign the next unplaced brick: pathfind to pile approach, then send TASK PICKUP."""
    if not unplaced or drone_name in in_flight:
        return
    brick_id = unplaced.popleft()
    in_flight[drone_name] = brick_id
    pile = PILE_POSITION
    approach = Vector3(pile.x, pile.y, TASK_ALT)
    task_msg = f"{drone_name} TASK PICKUP {brick_id} {pile.x:.4f} {pile.y:.4f}"
    _send_path_and_task(drone_name, approach, task_msg)
    print(f"[CTRL] Assigned brick {brick_id} to {drone_name}")


def send_place_leg(drone_name, brick_id):
    """After PICKUP: pathfind to target approach, then send TASK PLACE."""
    target = brick_targets[brick_id]
    tx, ty, tz = target.position.x, target.position.y, target.position.z
    approach = Vector3(tx, ty, TASK_ALT)
    task_msg = (
        f"{drone_name} TASK PLACE {brick_id} "
        f"{tx:.4f} {ty:.4f} {tz:.4f} {target.rotationZ:.4f}"
    )
    _send_path_and_task(drone_name, approach, task_msg)


while robot.step(timestep) != -1:
    t = robot.getTime()

    while receiver.getQueueLength() > 0:
        packet = receiver.getString()
        parts  = packet.split()

        if len(parts) == 9 and parts[1] == 'CURRENTPOS':
            name = parts[0]
            try:
                cur = Vector3.from_msg(' '.join(parts[2:5]))
                drone_positions[name] = cur
                # Assign a brick once the drone is airborne and unassigned
                if t >= 6.0 and cur.z > TAKEOFF_ALT * 0.7:
                    assign_brick(name)
            except ValueError:
                pass

        elif len(parts) == 3 and parts[1] == 'PICKUP':
            name = parts[0]
            try:
                brick_id = int(parts[2])
                send_place_leg(name, brick_id)
            except ValueError:
                pass

        elif len(parts) >= 3 and parts[1] == 'PLACE':
            name = parts[0]
            try:
                brick_id = int(parts[2])
                placed.add(brick_id)
                in_flight.pop(name, None)
                print(f"[CTRL] Brick {brick_id} placed by {name}. "
                      f"Progress: {len(placed)}/{len(brick_targets)}")
                assign_brick(name)
            except ValueError:
                pass

        receiver.nextPacket()

    if not takeoff_sent and t >= 2.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            emitter.send(f"{name} TAKEOFF".encode('utf-8'))
            print(f"[CTRL] Sent TAKEOFF to {name}")
        takeoff_sent = True
