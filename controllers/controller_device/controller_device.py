import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controller import Robot
from shared.vector3 import Vector3
from shared.voxel_world import VoxelWorld
from shared.pathfinder import find_path
from shared.path import Path
from shared.brick_parser import parse_brick_file, to_world_coords

BRICK_FILE = "../instructions/uni1.txt"

brick_states = to_world_coords(parse_brick_file(BRICK_FILE))
print(f"[CTRL] Loaded {len(brick_states)} brick states")

N_DRONES = 3
DEBUG_TARGETS = [
    Vector3(0.0, -2.0, 2.0),
    Vector3(0.0, -1.0, 2.0),
    Vector3(0.0,  0.0, 2.0),
    Vector3(0.0,  1.0, 2.0),
    Vector3(0.0,  2.0, 2.0),
]

# Navigable airspace bounds — adjust to match the simulation world layout.
# controller_device is a Robot (not Supervisor) so cannot query scene geometry at runtime.
WORLD_ORIGIN = Vector3(-5.0, -5.0, 0.0)
WORLD_SIZE   = Vector3(10.0, 10.0, 8.0)
VOXEL_SIZE   = 0.5
WAYPOINT_THRESHOLD = 0.5

robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice('emitter')

receiver = robot.getDevice('receiver')
receiver.enable(timestep)

voxel_world = VoxelWorld(WORLD_ORIGIN, WORLD_SIZE, VOXEL_SIZE)

drone_positions = {}   # {name: Vector3}  — last known position from CURRENTPOS
drone_paths     = {}   # {name: Path}     — currently reserved path

takeoff_sent = False
paths_sent   = False

while robot.step(timestep) != -1:
    t = robot.getTime()

    # Receive drone status messages to track positions and detect path completion.
    while receiver.getQueueLength() > 0:
        packet = receiver.getString()
        parts  = packet.split()
        if len(parts) == 9 and parts[1] == 'CURRENTPOS':
            name = parts[0]
            try:
                cur = Vector3.from_msg(' '.join(parts[2:5]))
                drone_positions[name] = cur
                # Free reservation once drone reaches its final waypoint.
                if name in drone_paths and len(drone_paths[name]) > 0:
                    final_wp = drone_paths[name][-1]
                    if Vector3.distance(cur, final_wp) < WAYPOINT_THRESHOLD:
                        voxel_world.mark_path_free(drone_paths[name])
                        del drone_paths[name]
                        print(f"[CTRL] Path reservation released for {name}")
            except ValueError:
                pass
        receiver.nextPacket()

    if not takeoff_sent and t >= 2.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            msg = f"{name} TAKEOFF"
            emitter.send(msg.encode('utf-8'))
            print(f"[CTRL] Sent: {msg}")
        takeoff_sent = True

    # After drones have reached takeoff altitude, compute and send A* paths.
    if not paths_sent and t >= 6.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            start = drone_positions.get(name)
            if start is None:
                print(f"[CTRL] No position known for {name}, skipping path")
                continue
            tgt = DEBUG_TARGETS[i]
            # Free any existing reservation before computing a new path.
            if name in drone_paths:
                voxel_world.mark_path_free(drone_paths[name])
            path = find_path(voxel_world, start, tgt)
            if path is None:
                print(f"[CTRL] No path found for {name}")
                continue
            voxel_world.mark_path_occupied(path)
            drone_paths[name] = path
            wp_strs = ' '.join(f"{wp.x},{wp.y},{wp.z}" for wp in path)
            msg = f"{name} PATH {wp_strs}"
            emitter.send(msg.encode('utf-8'))
            print(f"[CTRL] Sent path ({len(path)} waypoints) to {name}")
        paths_sent = True
