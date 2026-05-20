import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controller import Supervisor

from shared.debug_draw import DebugDraw
from shared.vector3 import Vector3

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
dbg = DebugDraw(supervisor)

while supervisor.step(timestep) != -1:
    #dbg.clear()
    #dbg.draw_wire_box(Vector3(1, 2, 0.5), Vector3(0.5, 0.5, 0.5), colour=(1, 0, 0))
