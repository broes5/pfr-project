from controller import Robot, Keyboard

print("Starting up drone controller...")

def clamp(value, low, high):
    return max(low, min(value, high))

robot = Robot()

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

imu = robot.getDevice('inertial unit'); imu.enable(timestep)
gps = robot.getDevice('gps');
gps.enable(timestep)
gyro = robot.getDevice('gyro');
gyro.enable(timestep)
kb = robot.getKeyboard();
kb.enable(timestep)

#camera_roll_motor = robot.getDevice('camera roll')
#camera_pitch_motor = robot.getDevice('camera pitch')

K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0
TARGET_ALT = 6.0 # metres — auto-takeoff
ALT_REACHED = 0.1 # metres — tolerance for "reached target"

LAND_SEQUENCE = [
(4.0, 2.0),
(2.0, 2.0),
(0.5, 2.0),
(0.0, 1.0), # ground — cut motors
]

# ── State machine constants
IDLE = 'IDLE'
TAKEOFF = 'TAKEOFF'
FLY = 'FLY'
LAND = 'LAND'
DONE = 'DONE'

# ── State variables
state = IDLE
target_altitude = 0.0
land_stage = 0
land_stage_timer = 0.0

# ── Wait 1 second for physics to settle
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break
print('t=Takeoff l=Auto-land Arrows=fly Shift+Arrows=altitude/strafe')

