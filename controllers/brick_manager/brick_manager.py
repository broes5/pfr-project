import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controller import Supervisor
from shared.vector3 import Vector3
from shared.brick_state import BrickState
from shared.brick_parser import parse_brick_file, to_world_coords
from shared.brick_placer import BrickPool
from shared.pile_layout import pile_pos_for_brick, PILE_POSITION

BRICK_FILE = "../instructions/uni1.txt"
N_DRONES = 3
DRONE_SLOT_INIT_DELAY_MS = 1000

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

receiver = supervisor.getDevice('receiver')
receiver.enable(timestep)

raw_bricks = parse_brick_file(BRICK_FILE)
brick_targets = to_world_coords(raw_bricks)

pool = BrickPool(supervisor)
pool.pre_spawn(len(brick_targets), physics=False)

for brick_id in range(len(brick_targets)):
    pos = pile_pos_for_brick(brick_id) or PILE_POSITION
    pool.spawn(brick_id, BrickState(pos, 0.0))

print(f"[BrickManager] Spawned {len(brick_targets)} bricks in pile grid around {PILE_POSITION.x:.1f},{PILE_POSITION.y:.1f}")


def _find_drone_node(drone_name):
    root = supervisor.getRoot().getField("children")
    for i in range(root.getCount()):
        node = root.getMFNode(i)
        f = node.getField("name")
        if f and f.getSFString() == drone_name:
            return node
    return None


drone_brick_slots = {}  # drone_name -> brickSlot MFNode proto field
carrying = {}           # drone_name -> brick_id or None

slots_initialized = False
init_elapsed = 0

while supervisor.step(timestep) != -1:
    init_elapsed += timestep

    if not slots_initialized and init_elapsed >= DRONE_SLOT_INIT_DELAY_MS:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            node = _find_drone_node(name)
            if node:
                slot = node.getField("brickSlot")
                if slot:
                    drone_brick_slots[name] = slot
                    carrying[name] = None
                else:
                    print(f"[BrickManager] WARNING: brickSlot field not found on {name}")
            else:
                print(f"[BrickManager] WARNING: {name} not found in scene")
        slots_initialized = True
        print(f"[BrickManager] Slots cached for: {list(drone_brick_slots.keys())}")

    while receiver.getQueueLength() > 0:
        packet = receiver.getString()
        parts = packet.split()
        receiver.nextPacket()

        if len(parts) < 3:
            continue

        sender = parts[0]
        verb = parts[1]

        if verb == 'PICKUP' and len(parts) == 3:
            try:
                brick_id = int(parts[2])
            except ValueError:
                continue
            slot = drone_brick_slots.get(sender)
            if slot is None:
                print(f"[BrickManager] PICKUP from unregistered drone: {sender}")
                continue
            pool.despawn(brick_id)
            slot.importMFNodeFromString(-1, 'BrickStill { translation 0 0 0 rotation 0 0 1 0 }')
            carrying[sender] = brick_id
            print(f"[BrickManager] {sender} picked up brick {brick_id}")

        elif verb == 'PLACE' and len(parts) == 7:
            try:
                brick_id = int(parts[2])
                x, y, z = float(parts[3]), float(parts[4]), float(parts[5])
                rot = float(parts[6])
            except ValueError:
                print(f"[BrickManager] Malformed PLACE: {packet}")
                continue
            slot = drone_brick_slots.get(sender)
            if slot is None:
                print(f"[BrickManager] PLACE from unregistered drone: {sender}")
                continue
            if slot.getCount() > 0:
                slot.getMFNode(0).remove()
            pool.spawn(brick_id, BrickState(Vector3(x, y, z), rot))
            carrying[sender] = None
            print(f"[BrickManager] {sender} placed brick {brick_id} at ({x:.2f},{y:.2f},{z:.2f})")
