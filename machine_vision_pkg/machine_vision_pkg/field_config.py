"""Shared field geometry: the single source of truth for both localization_node.py
(which uses it to triangulate the robot's position from the 4 corner tags) and
ball_pursuit_node.py (which uses it to know where the goal already is, without needing
to detect it itself).

Field is a square with an AprilTag (36h11) mounted at each corner, each one
rotated so it faces straight at the field center -- that's what makes
tag_facing = atan2(CENTER - tag_y, CENTER - tag_x) in localization_node.py a
valid stand-in for "the tag's known orientation on the field." Two more tags
mark the goals, at the midpoints of the x=0 and x=FIELD_SIZE edges, each also
facing into the field (perpendicular to its own goal line).

If the physical field's size or any tag's position changes, change it here --
both nodes read it from this one place so they can't silently drift apart.
"""

FIELD_SIZE = 3.6576      # meters (12 ft, 2x the original 6 ft)
TAG_SIZE = 0.075         # meters, edge length of the tag's black square
TAG_HEIGHT = 0.0635      # meters, tag center height off the ground
CAMERA_HEIGHT = 0.2667   # meters, Neato camera height off the ground
CENTER = FIELD_SIZE / 2

# corners 0-3, goals 4-5
TAG_POSITIONS = {
    0: (0.0, 0.0),
    1: (FIELD_SIZE, 0.0),
    2: (FIELD_SIZE, FIELD_SIZE),
    3: (0.0, FIELD_SIZE),
    4: (0.0, CENTER),
    5: (FIELD_SIZE, CENTER),
}
