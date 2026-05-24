import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from collections import deque
from controller import Robot
from shared.vector3 import Vector3
from shared.voxel_world import VoxelWorld
from shared.pathfinder import find_path
from shared.brick_parser import parse_brick_file, to_world_coords
from shared.pile_layout import (
    configure, pile_pos_for_brick, pile_layer_for_brick,
    pile_layer_size, pile_layer_count_for_layer, pile_layers_for_count,
    PICKUP_HOVER_OFFSET_A, PICKUP_HOVER_OFFSET_B, PILE_POSITION,
)
from shared.config import (
    N_DRONES, BRICK_FILE,
    TASK_ALT, TAKEOFF_ALT, PICKUP_ALT, PLACE_ALT_OFFSET,
)

WORLD_ORIGIN = Vector3(-8.0, -8.0, 0.0)
WORLD_SIZE   = Vector3(16.0, 16.0, 8.0)
VOXEL_SIZE   = 0.35

brick_targets = to_world_coords(parse_brick_file(BRICK_FILE))
configure(len(brick_targets))
print(f"[CTRL] Loaded {len(brick_targets)} brick targets")

robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(timestep)

voxel_world = VoxelWorld(WORLD_ORIGIN, WORLD_SIZE, VOXEL_SIZE)

# ── Layer gate: tracks how many bricks per layer have been picked up
_layer_size      = pile_layer_size()
_n_layers        = pile_layers_for_count(len(brick_targets))
pile_layer_taken = [0] * _n_layers

# Brick lifecycle tracking
unplaced  = deque(range(len(brick_targets)))  # brick_ids not yet placed
placed    = set()                              # brick_ids successfully placed
in_flight = {}                                 # drone_name -> brick_id being carried

drone_positions       = {}  # drone_name -> last known Vector3
drone_first_positions = {}  # drone_name -> first position ever reported (used as rest bay XY)
drone_reserved        = {}  # drone_name -> list of waypoints currently marked in voxel_world
waiting               = set()  # drone names blocked on a failed pathfind, awaiting a retry
waiting_since         = {}  # drone_name -> sim time when it was added to waiting
pile_in_progress      = {}  # drone_name -> layer_idx of the pickup in progress
resting               = set()  # drone names that have been routed to their rest bay

# After this many seconds without a path clearing, force a retry regardless.
STUCK_TIMEOUT = 30.0

takeoff_sent = False


def _path_msg(drone_name, start, goal, yaw=None, _collect=None):
    """Compute A* path, mark it occupied, send PATH message. Returns True on success.

    Frees the 3x3x3 neighbourhood of start before pathfinding so A* can expand
    from the shared endpoint of the previous leg (which was fully claimed).
    On failure the neighbourhood is restored exactly as it was.
    """
    freed_nbrs = voxel_world.free_neighborhood(start, radius=1)
    path = find_path(voxel_world, start, goal)
    if path and len(path) > 0:
        claimed = voxel_world.mark_path_neighborhood_occupied(path)
        if _collect is not None:
            _collect.extend(claimed)
        wp_strs = ' '.join(f"{wp.x},{wp.y},{wp.z}" for wp in path)
        yaw_prefix = f"YAW:{yaw:.4f} " if yaw is not None else ""
        emitter.send(f"{drone_name} PATH {yaw_prefix}{wp_strs}".encode('utf-8'))
        yaw_info = f" yaw={yaw:.1f}°" if yaw is not None else ""
        print(f"[CTRL] PATH{yaw_info} ({len(path)} wps) ({start.x:.2f},{start.y:.2f},{start.z:.2f}) → ({goal.x:.2f},{goal.y:.2f},{goal.z:.2f})")
        return True
    else:
        voxel_world.restore_voxels(freed_nbrs)
        print(f"[CTRL] No path ({start.x:.2f},{start.y:.2f},{start.z:.2f}) → ({goal.x:.2f},{goal.y:.2f},{goal.z:.2f}) — blocked")
        return False


def _retry_waiting(t=0.0):
    """Re-attempt path assignment for every drone that was previously blocked."""
    for drone in list(waiting):
        waiting.discard(drone)
        waiting_since.pop(drone, None)
        if drone in in_flight:
            send_place_leg(drone, in_flight[drone], t)
        else:
            assign_brick(drone, t)


def _free_reservation(drone_name, t=0.0):
    """Unmark all voxels reserved for this drone, then retry any blocked drones."""
    freed = drone_reserved.pop(drone_name, [])
    for wp in freed:
        voxel_world.mark_free(wp)
    if freed:
        _retry_waiting(t)


def _next_assignable_brick():
    """Peek at unplaced queue; return brick_id only if its pile layer is open.

    Layer gating enforces structural integrity: a drone can only pick up a brick from
    layer N once every brick in layers 0..N-1 has already been lifted from the pile.
    Without this, upper-layer bricks could be placed before the lower layer is complete,
    and collapsing the pile would scatter bricks that drones are still trying to reach.

    Returns None if the queue is empty or the layer gate is closed.
    """
    if not unplaced:
        return None
    brick_id = unplaced[0]
    li = pile_layer_for_brick(brick_id)
    if li >= _n_layers:
        return brick_id   # brick beyond computed layers — no gate to check
    for l in range(li):
        if pile_layer_taken[l] < pile_layer_count_for_layer(l):
            return None   # upper layer still has bricks to pick up
    return brick_id


def assign_brick(drone_name, t=0.0):
    """Assign the next unplaced brick: 3-leg path (ascend → cruise → descend) to pickup."""
    cur = drone_positions.get(drone_name)
    if cur is None or drone_name in in_flight or drone_name in waiting or drone_name in resting:
        return

    brick_id = _next_assignable_brick()
    if brick_id is None:
        if unplaced:
            waiting.add(drone_name)
            waiting_since[drone_name] = t
            print(f"[CTRL] {drone_name}: waiting for pile layer to complete")
        else:
            # No bricks remain — pathfind to rest bay, then queue a LAND task
            first = drone_first_positions[drone_name]
            rest = Vector3(first.x, first.y, TASK_ALT)
            p1 = Vector3(cur.x, cur.y, TASK_ALT)
            _free_reservation(drone_name, t)
            wps = []
            ok = (_path_msg(drone_name, cur, p1, _collect=wps) and
                  _path_msg(drone_name, p1, rest, _collect=wps))
            if ok:
                drone_reserved[drone_name] = wps
                resting.add(drone_name)
                emitter.send(f"{drone_name} TASK LAND".encode('utf-8'))
                print(f"[CTRL] {drone_name}: no bricks left — pathing to rest bay, LAND queued")
            else:
                print(f"[CTRL] {drone_name}: path to rest bay blocked, will retry")
        return

    pile_pos = pile_pos_for_brick(brick_id)   # bottom of brick
    if pile_pos is None:
        print(f"[CTRL] No pile position for brick {brick_id}")
        return

    layer_idx = pile_layer_for_brick(brick_id)
    _free_reservation(drone_name, t)
    unplaced.popleft()   # commit — position is deterministic from brick_id

    hover = Vector3(pile_pos.x, pile_pos.y, pile_pos.z + PICKUP_HOVER_OFFSET_B)

    p1_goal = Vector3(cur.x,    cur.y,    TASK_ALT)
    p2_goal = Vector3(hover.x,  hover.y,  TASK_ALT)
    p3_goal = Vector3(pile_pos.x, pile_pos.y, pile_pos.z + PICKUP_HOVER_OFFSET_A)
    p4_goal = hover

    hover   = p4_goal

    wps = []
    ok = (_path_msg(drone_name, cur,     p1_goal,          _collect=wps) and
          _path_msg(drone_name, p1_goal, p2_goal,          _collect=wps) and
          _path_msg(drone_name, p2_goal, p3_goal, yaw=0.0, _collect=wps) and
          _path_msg(drone_name, p3_goal, p4_goal, yaw=0.0, _collect=wps))

    if not ok:
        for wp in wps:
            voxel_world.mark_free(wp)
        unplaced.appendleft(brick_id)
        waiting.add(drone_name)
        waiting_since[drone_name] = t
        print(f"[CTRL] {drone_name}: path to pile blocked, waiting")
        return

    drone_reserved[drone_name] = wps
    in_flight[drone_name] = brick_id
    pile_in_progress[drone_name] = layer_idx
    emitter.send(
        f"{drone_name} TASK PICKUP {brick_id} "
        f"{hover.x:.4f} {hover.y:.4f} {hover.z:.4f} 0.0"
        .encode('utf-8')
    )
    print(f"[CTRL] Assigned brick {brick_id} to {drone_name} → pile layer {layer_idx} "
          f"({pile_pos.x:.2f},{pile_pos.y:.2f},{pile_pos.z:.2f})")


def send_place_leg(drone_name, brick_id, t=0.0):
    """After PICKUP: 3-leg path (ascend → cruise → descend) to place position.

    If any leg is blocked, this drone is added to `waiting` for a later retry.
    The TASK PLACE message is only sent once a valid path is found.
    """
    _free_reservation(drone_name, t)
    cur = drone_positions.get(drone_name)
    target = brick_targets[brick_id]
    tx, ty, tz = target.position.x, target.position.y, target.position.z
    place_z = max(tz + PLACE_ALT_OFFSET, PICKUP_ALT)
    place_yaw = target.rotationZ

    if cur is None:
        print(f"[CTRL] No position for {drone_name}, PLACE task will use fallback")
    else:
        p1_goal = Vector3(cur.x, cur.y, TASK_ALT)
        p2_goal = Vector3(tx, ty, TASK_ALT)
        p3_goal = Vector3(tx, ty, place_z)
        wps = []
        ok = (_path_msg(drone_name, cur,     p1_goal,                   _collect=wps) and
              _path_msg(drone_name, p1_goal, p2_goal,                   _collect=wps) and
              _path_msg(drone_name, p2_goal, p3_goal, yaw=place_yaw,   _collect=wps))

        if not ok:
            for wp in wps:
                voxel_world.mark_free(wp)
            waiting.add(drone_name)
            waiting_since[drone_name] = t
            print(f"[CTRL] {drone_name}: path to place brick {brick_id} blocked, waiting")
            return

        drone_reserved[drone_name] = wps

    emitter.send((
        f"{drone_name} TASK PLACE {brick_id} "
        f"{tx:.4f} {ty:.4f} {tz:.4f} {target.rotationZ:.4f}"
    ).encode('utf-8'))


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
                if name not in drone_first_positions:
                    drone_first_positions[name] = cur
                # Wait until t≥6 s and the drone is at 70 % of cruise altitude before
                # assigning work.  The 6 s window covers the physics-settle delay plus
                # the time all four drones need to reach TAKEOFF_ALT simultaneously,
                # preventing path collisions during the initial climb.
                if t >= 6.0 and cur.z > TAKEOFF_ALT * 0.7:
                    assign_brick(name, t)
            except ValueError:
                pass

        elif len(parts) == 3 and parts[1] == 'PICKUP':
            name = parts[0]
            try:
                brick_id = int(parts[2])
                if name in pile_in_progress:
                    li = pile_in_progress.pop(name)
                    if li < _n_layers:
                        pile_layer_taken[li] += 1
                        layer_count = pile_layer_count_for_layer(li)
                        done = pile_layer_taken[li] == layer_count
                        print(f"[CTRL] Pile layer {li}: {pile_layer_taken[li]}/{layer_count} picked up"
                              + (" — layer complete" if done else ""))
                send_place_leg(name, brick_id, t)
            except ValueError:
                pass

        elif len(parts) >= 3 and parts[1] == 'PLACE':
            name = parts[0]
            try:
                brick_id = int(parts[2])
                placed.add(brick_id)
                in_flight.pop(name, None)
                voxel_world.mark_occupied(brick_targets[brick_id].position)
                print(f"[CTRL] Brick {brick_id} placed by {name}. "
                      f"Progress: {len(placed)}/{len(brick_targets)}")
                if len(placed) == len(brick_targets):
                    print(f"[CTRL] All {len(brick_targets)} bricks placed — sending LAND to all drones")
                    for i in range(N_DRONES):
                        emitter.send(f"BrickDrone_{i} LAND".encode('utf-8'))
                else:
                    assign_brick(name, t)
            except ValueError:
                pass

        receiver.nextPacket()

    if not takeoff_sent and t >= 2.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            emitter.send(f"{name} TAKEOFF".encode('utf-8'))
            print(f"[CTRL] Sent TAKEOFF to {name}")
        takeoff_sent = True

    # Force-retry any drone that has been stuck in waiting longer than STUCK_TIMEOUT.
    for drone in list(waiting):
        if t - waiting_since.get(drone, t) >= STUCK_TIMEOUT:
            print(f"[CTRL] {drone} stuck for {STUCK_TIMEOUT:.0f}s — forcing retry")
            waiting.discard(drone)
            waiting_since.pop(drone, None)
            if drone in in_flight:
                send_place_leg(drone, in_flight[drone], t)
            else:
                assign_brick(drone, t)
