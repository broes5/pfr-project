from controller import Robot, Keyboard
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

# ── State machine constants
IDLE, TAKEOFF, FLY, LAND, DONE = 'IDLE', 'TAKEOFF', 'FLY', 'LAND', 'DONE'

state = IDLE
land_stage = 0
land_stage_timer = 0.0
print_counter = 0  # To prevent console flooding

# target altitude 
targetAlt = 0.0

# target rotation in degrees
targetYaw = 0.0

#target position
targetX = 0.0
targetY = 0.0

# ── Motors
fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# ── Sensors
imu = robot.getDevice('inertial unit'); imu.enable(timestep)
gyro = robot.getDevice('gyro'); gyro.enable(timestep)
gps = robot.getDevice('gps'); gps.enable(timestep)
kb = robot.getKeyboard(); kb.enable(timestep)

# ── TUNED CONSTANTS ──────────────────
K_VERTICAL_THRUST = 68.5 # thrust required to counteract gravity
K_VERTICAL_OFFSET = 0.6 # offset to hover on target altitude due to the drones mass
K_VERTICAL_P = 3.0 # rate of aggression towards reaching target altitude

# Attitude (Tilt) Constants
K_ROLL_P = 15.0 # rate of aggression towards reaching roll angle
K_PITCH_P = 15.0 # rate of aggression towards reaching pitch angle
K_GYRO_D = 3.0 # Tilt damper, uses gyro to stop drone from rotating or swinging around like a pendulum

# Position (GPS) Constants
K_POS_P = 0.3 # Rate of attraction to coordinates on the horizontal plane
K_POS_D = 0.6 # Slows drone down so it does not overshoot
K_POS_I = 0.3 # Prevents steady state error from attraction to correct coordinates not being high enough to move the drone

# Rotation Constants
K_YAW_P = 2.0

V_FILTER = 0.1
TAKEOFF_ALT = 3.0 # default altitude to take off to
ALT_REACHED = 0.1 # Bracket  of error allowance for target altitude being reached

# ── Memory Variables
prevX, prevY = 0.0, 0.0
f_xVel, f_yVel = 0.0, 0.0 
i_ex, i_ey = 0.0, 0.0 # Integral accumulators

# ── Auto-landing sequence: (target_altitude_m,hold_seconds)
LAND_SEQUENCE = [
    (3.0, 3.0),
    (2.0, 3.0),
    (0.5, 3.0),
    (0.0, 2.0), # ground — cut motors
]

def clamp(value, low, high):
    return max(low, min(value, high))

# ── Wait for physics to settle
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        vals = gps.getValues()
        prevX, prevY = vals[0], vals[1]
        # Initialize targetYaw to current orientation
        _, _, start_yaw = imu.getRollPitchYaw()
        targetYaw = math.degrees(start_yaw) % 360.0
        print(f'Physics settled. Initial GPS: {prevX:.2f}, {prevY:.2f}')
        break

print('\n──────────────────────────────────────────────────\n                  CONTROLS\n──────────────────────────────────────────────────\nT = Takeoff\n────────────────   While Flying   ────────────────\nL = Land\nWASD = Move Target\nArrow keys = Change Target Altitude/Yaw')

# ──────────────────────────────────────────────────
#                Main Loop
# ──────────────────────────────────────────────────

while robot.step(timestep) != -1:
    t = robot.getTime()
    
    # 1. Sensors
    gps_vals = gps.getValues()
    xPos, yPos, gpsAlt = gps_vals[0], gps_vals[1], gps_vals[2]
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_vel, pitch_vel, yaw_vel = gyro.getValues()
    
    # 2. Filtered Velocity Calculation
    raw_xVel = (xPos - prevX) / dt
    raw_yVel = (yPos - prevY) / dt
    f_xVel = (f_xVel * (1 - V_FILTER)) + (raw_xVel * V_FILTER)
    f_yVel = (f_yVel * (1 - V_FILTER)) + (raw_yVel * V_FILTER)
    prevX, prevY = xPos, yPos
    roll_d = pitch_d = yaw_d = 0.0
    
    # 3. Keyboard Input
    key = kb.getKey()
    while key > 0:
        if key == ord('W') and state == FLY: 
            targetX -= 0.1
            print(f'[Input] Target X moved to: {targetX:.2f}')
        elif key == ord('S') and state == FLY: 
            targetX += 0.1
            print(f'[Input] Target X moved to: {targetX:.2f}')
        elif key == ord('D') and state == FLY: 
            targetY += 0.1
            print(f'[Input] Target Y moved to: {targetY:.2f}')
        elif key == ord('A') and state == FLY: 
            targetY -= 0.1
            print(f'[Input] Target Y moved to: {targetY:.2f}')
            
        # Yaw Control (Changes target angle)
        elif key == Keyboard.RIGHT and state == FLY: 
            targetYaw = (targetYaw - 2.0) % 360.0
        elif key == Keyboard.LEFT and state == FLY: 
            targetYaw = (targetYaw + 2.0) % 360.0

        elif key == Keyboard.UP and state == FLY: 
            targetAlt += 0.1
            print(f'[Input] Target Alt: {targetAlt:.2f}')
        elif key == Keyboard.DOWN and state == FLY: 
            targetAlt -= 0.1
            print(f'[Input] Target Alt: {targetAlt:.2f}')
        elif key == ord('T') and state == IDLE:
            targetAlt = TAKEOFF_ALT
            state = TAKEOFF
            print(f'>> TAKEOFF sequence initiated to {TAKEOFF_ALT}m')
        elif key == ord('L') and state == FLY:
            state = LAND
            land_stage = 0
            land_stage_timer = t
            print(f'>> AUTO-LAND initiated at current position')
        key = kb.getKey() 
        
    # 4. State Management & Periodic Debug Printing
    print_counter += 1
    if state == TAKEOFF and abs(gpsAlt - TAKEOFF_ALT) < ALT_REACHED:
        targetX = xPos
        targetY = yPos
        targetYaw = yaw
        state = FLY
        print(f'>> TAKEOFF COMPLETE. Holding position at X:{targetX:.2f}, Y:{targetY:.2f}. Switching to fly state')
    
    if state == FLY and print_counter % 100 == 0:
        # Periodic status update
        actual_yaw_deg = math.degrees(yaw) % 360.0
        print(f'Target: (X: {targetX:.2f}, Y: {targetY:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {targetAlt:.2f})\nActual: (X: {xPos:.2f}, Y: {yPos:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {gpsAlt:.2f}')
            
    # 5. Position Controller (World to Body)
    cosY, sinY = math.cos(yaw), math.sin(yaw)
    ex, ey = (targetX - xPos), (targetY - yPos)
    
    body_ex = ex * cosY + ey * sinY
    body_ey = ey * cosY - ex * sinY
    body_vx = f_xVel * cosY + f_yVel * sinY
    body_vy = f_yVel * cosY - f_xVel * sinY

    if state in [FLY, TAKEOFF, LAND]:
        # Update Integrals with clamping to prevent Windup
        i_ex = clamp(i_ex + body_ex * dt, -0.5, 0.5)
        i_ey = clamp(i_ey + body_ey * dt, -0.5, 0.5)

        # PID Formula: (Proportional + Integral) - Derivative
        pitch_cmd = (K_POS_P * body_ex) + (K_POS_I * i_ex) - (K_POS_D * body_vx)
        roll_cmd  = -((K_POS_P * body_ey) + (K_POS_I * i_ey) - (K_POS_D * body_vy))
        
        desired_pitch = clamp(pitch_cmd + pitch_d, -0.3, 0.3)
        desired_roll = clamp(roll_cmd + roll_d, -0.3, 0.3)
        
        # 6. Altitude Controller
        v_error = clamp(targetAlt - gpsAlt + K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = K_VERTICAL_P * (v_error ** 3)
    
        # 7. Attitude Controller with Gyro Damping
        roll_input = K_ROLL_P * clamp(roll - desired_roll, -1.0, 1.0) + (roll_vel * K_GYRO_D)
        pitch_input = K_PITCH_P * clamp(pitch - desired_pitch, -1.0, 1.0) + (pitch_vel * K_GYRO_D)
        
        target_rad = math.radians(targetYaw)
        # Shift 0-2pi to -pi to pi to match IMU
        if target_rad > math.pi: target_rad -= 2.0 * math.pi
        
        yaw_error = target_rad - yaw
        # Shortest path wrap-around
        while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi: yaw_error += 2.0 * math.pi
        
        yaw_input = (K_YAW_P * yaw_error) - (yaw_vel * K_GYRO_D)
        
        # 8. Motor Mixer
        fl_v = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        fr_v = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rl_v = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rr_v = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input
    
        # 9. Set Velocities
        fl.setVelocity( fl_v); fr.setVelocity(-fr_v)
        rl.setVelocity(-rl_v); rr.setVelocity( rr_v)
    else:
        # IDLE or DONE: Keep motors off
        fl.setVelocity(0.0); fr.setVelocity(0.0)
        rl.setVelocity(0.0); rr.setVelocity(0.0)
    
     # ── LAND — staged descent
    if state == LAND:
        if land_stage < len(LAND_SEQUENCE):
            tgt, hold = LAND_SEQUENCE[land_stage]
            targetAlt = tgt
            
            # Advance only when altitude is reached AND hold time has elapsed
            if abs(gpsAlt - tgt) < ALT_REACHED and (t- land_stage_timer) > hold:
                print(f'>> Land stage {land_stage + 1} complete')
                land_stage += 1
                land_stage_timer = t # reset timer for the next stage
        else:
            state = DONE # all stages complete — cut motors
   
    # ── DONE — cut all motors and skip the rest of the loop
    if state == DONE:
        for m in [fl, fr, rl, rr]:
            m.setVelocity(0.0)
            continue # skip PID and mixer this step