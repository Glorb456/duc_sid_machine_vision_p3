""" Field geometry shared by localization_node and ball_pursuit_node. The field
    is a square with a 36h11 AprilTag in each corner facing the center, plus
    one tag per goal at the midpoints of the x=0 and x=FIELD_SIZE edges. """

FIELD_SIZE = 3.6576      # meters (12 ft)
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
