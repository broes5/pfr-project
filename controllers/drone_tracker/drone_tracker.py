import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controller import Supervisor
from shared.debug_draw import DebugDraw
from shared.vector3 import Vector3
from shared.path import Path
from shared.color import Color

DRONE_COLORS = [
    Color.red,
    Color.green,
    Color.blue,
    Color.yellow,
    Color.cyan,
    Color.magenta,
]

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

receiver = supervisor.getDevice('receiver')
receiver.enable(timestep)

dbg = DebugDraw(supervisor, '__DRONE_TRACKER__')

drone_states = {}  # {name: (Vector3 current, Vector3 target)}

while supervisor.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        packet = receiver.getString()
        parts = packet.split()
        # Status report: "BrickDrone_N CURRENTPOS x y z TARGETPOS x y z"
        if len(parts) == 9 and parts[1] == 'CURRENTPOS' and parts[5] == 'TARGETPOS':
            name = parts[0]
            try:
                cur = Vector3.from_msg(' '.join(parts[2:5]))
                tgt = Vector3.from_msg(' '.join(parts[6:9]))
                drone_states[name] = (cur, tgt)
            except ValueError:
                pass
        receiver.nextPacket()

    dbg.clear()
    for i, name in enumerate(sorted(drone_states)):
        color = DRONE_COLORS[i % len(DRONE_COLORS)]
        cur, tgt = drone_states[name]
        dbg.draw_line(cur, tgt, color)