from .vector3 import Vector3
from .color import Color
from .path import Path


class DebugDraw:
    """Draws debug geometry into the Webots scene via the Supervisor API.

    Requires the controller to have supervisor=TRUE in the world file.

    Usage:
        dbg = DebugDraw(supervisor)
        dbg.draw_path(path)
        dbg.draw_wire_box(centre, size, Color.red)
        dbg.clear()
    """

    def __init__(self, supervisor, name='__DEBUG__'):
        self._supervisor = supervisor
        self._group = supervisor.getFromDef(name)
        if self._group is None:
            root_children = supervisor.getRoot().getField('children')
            root_children.importMFNodeFromString(-1, f'DEF {name} Group {{ children [] }}')
            self._group = supervisor.getFromDef(name)
        self._children = self._group.getField('children')

    def clear(self):
        """Remove all debug geometry."""
        while self._children.getCount() > 0:
            self._children.removeMF(0)

    def draw_line(self, a: Vector3, b: Vector3, colour: Color = None):
        if colour is None:
            colour = Color.red
        self._children.importMFNodeFromString(-1, f'''Shape {{
  appearance Appearance {{ material Material {{ emissiveColor {colour.r} {colour.g} {colour.b} }} }}
  geometry IndexedLineSet {{
    coord Coordinate {{ point [ {a.x} {a.y} {a.z}  {b.x} {b.y} {b.z} ] }}
    coordIndex [ 0 1 -1 ]
  }}
}}''')

    def draw_wire_box(self, centre: Vector3, size: Vector3, colour: Color = None):
        if colour is None:
            colour = Color.red
        hx, hy, hz = size.x * 0.5, size.y * 0.5, size.z * 0.5
        cx, cy, cz = centre.x, centre.y, centre.z
        pts = [
            (cx-hx, cy-hy, cz-hz), (cx+hx, cy-hy, cz-hz),
            (cx+hx, cy+hy, cz-hz), (cx-hx, cy+hy, cz-hz),
            (cx-hx, cy-hy, cz+hz), (cx+hx, cy-hy, cz+hz),
            (cx+hx, cy+hy, cz+hz), (cx-hx, cy+hy, cz+hz),
        ]
        pts_str = '  '.join(f'{x} {y} {z}' for x, y, z in pts)
        self._children.importMFNodeFromString(-1, f'''Shape {{
  appearance Appearance {{ material Material {{ emissiveColor {colour.r} {colour.g} {colour.b} }} }}
  geometry IndexedLineSet {{
    coord Coordinate {{ point [ {pts_str} ] }}
    coordIndex [ 0 1 -1  1 2 -1  2 3 -1  3 0 -1
                 4 5 -1  5 6 -1  6 7 -1  7 4 -1
                 0 4 -1  1 5 -1  2 6 -1  3 7 -1 ]
  }}
}}''')

    def draw_path(self, path: Path, colour: Color = None):
        if colour is None:
            colour = Color.green
        for i in range(len(path) - 1):
            self.draw_line(path[i], path[i + 1], colour)
