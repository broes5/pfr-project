from drone_setup import *

# ──────────────────────────────────────────────────
#                Main Loop
# ──────────────────────────────────────────────────
while robot.step(timestep) != -1:
    t = robot.getTime()
    
    # 1. Sensors
    gps_vals = gps.getValues()
    currentPos = Vector3(gps_vals[0], gps_vals[1], gps_vals[2])
    
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_vel, pitch_vel, yaw_vel = gyro.getValues()
    
    # data send / receive
    if receiver and receiver.getQueueLength() > 0:
        while receiver.getQueueLength() > 0:
            # assume incoming string format is: "TARGET X Y Z"
            packet = receiver.getString()
            parts = packet.split()
            
            if len(parts) == 5 and parts[0] == drone_name and parts[1] == 'TARGET':
                try:
                    targetPos.x = float(parts[2])
                    targetPos.y = float(parts[3])
                    targetPos.z = float(parts[4])
                    print(f"[COMMS] New target received: X:{targetPos.x:.2f} Y:{targetPos.y:.2f} Z:{targetPos.z:.2f}")
                except:
                    print(f"[COMMS] Error parsing coordinate data.")
            elif len(parts) == 2 and parts[0] == drone_name and parts[1] == 'TAKEOFF':
                if state == IDLE:
                    targetPos.x = currentPos.x
                    targetPos.y = currentPos.y
                    targetPos.z = TAKEOFF_ALT
                    targetYaw = yaw
                    state = TAKEOFF
                    print(f'>> [COMMS] TAKEOFF command received')
            receiver.nextPacket()
    # send drone's current status (current and target location) at twice the speed of the print counter
    if emitter and print_counter % 50 == 0:
        status_msg = f"{drone_name} CURRENTPOS {currentPos.x:.2f} {currentPos.y:.2f} {currentPos.z:.2f} TARGETPOS {targetPos.x:.2f} {targetPos.y:.2f} {targetPos.z:.2f}"
        emitter.send(status_msg.encode('utf-8'))
    
    # 2. Filtered Velocity Calculation
    raw_xVel = (currentPos.x - prevPos.x) / dt
    raw_yVel = (currentPos.y - prevPos.y) / dt
    raw_zVel = (currentPos.z - prevPos.z) / dt
    
    f_xVel = (f_xVel * (1 - V_FILTER)) + (raw_xVel * V_FILTER)
    f_yVel = (f_yVel * (1 - V_FILTER)) + (raw_yVel * V_FILTER)
    f_zVel = (f_zVel * (1 - V_FILTER)) + (raw_zVel * V_FILTER)
    
    prevPos = currentPos
    roll_d = pitch_d = yaw_d = 0.0
    
    # 3. Keyboard Input
    key = kb.getKey()
    while key > 0:
        if key == ord('W') and state == FLY: 
            targetPos.x += 0.1
            print(f'[Input] Target X moved to: {targetPos.x:.2f}')
        elif key == ord('S') and state == FLY: 
            targetPos.x -= 0.1
            print(f'[Input] Target X moved to: {targetPos.x:.2f}')
        elif key == ord('D') and state == FLY: 
            targetPos.y -= 0.1
            print(f'[Input] Target Y moved to: {targetPos.y:.2f}')
        elif key == ord('A') and state == FLY: 
            targetPos.y += 0.1
            print(f'[Input] Target Y moved to: {targetPos.y:.2f}')
            
        # Yaw Control (Changes target angle)
        elif key == Keyboard.RIGHT and state == FLY: 
            targetYaw = (targetYaw - 0.5) % 360.0
        elif key == Keyboard.LEFT and state == FLY: 
            targetYaw = (targetYaw + 0.5) % 360.0

        # Altitude Control (changes target altitude)
        elif key == Keyboard.UP and state == FLY: 
            targetPos.z += 0.1
            print(f'[Input] Target Alt: {targetPos.z:.2f}')
        elif key == Keyboard.DOWN and state == FLY: 
            targetPos.z -= 0.1
            print(f'[Input] Target Alt: {targetPos.z:.2f}')
        
        # special keys (Takeoff and landing)
        elif key == ord('T') and state == IDLE:
            targetPos.x = currentPos.x
            targetPos.y = currentPos.y
            targetPos.z = TAKEOFF_ALT
            targetYaw = yaw
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
    if state == TAKEOFF and abs(currentPos.z - TAKEOFF_ALT) < ALT_REACHED:
        targetPos.x = currentPos.x
        targetPos.y = currentPos.y
        state = FLY
        print(f'>> TAKEOFF COMPLETE. Holding position at X:{targetPos.x:.2f}, Y:{targetPos.y:.2f}. Switching to fly state')
    
    if state == FLY and print_counter % 100 == 0:
        # Periodic status update
        actual_yaw_deg = math.degrees(yaw) % 360.0
        print(f'Target: (X: {targetPos.x:.2f}, Y: {targetPos.y:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {targetPos.z:.2f})\nActual: (X: {currentPos.x:.2f}, Y: {currentPos.y:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {currentPos.z:.2f}')
            
    # 5. Position Controller (World to Body)
    cosY, sinY = math.cos(yaw), math.sin(yaw)
    ex = (targetPos.x - currentPos.x)
    ey = (targetPos.y - currentPos.y)
    
    body_vx = f_xVel * cosY + f_yVel * sinY
    body_vy = f_yVel * cosY - f_xVel * sinY
    
    # Calculate TRUE raw errors in drone body frame
    body_ex = ex * cosY + ey * sinY
    body_ey = ey * cosY - ex * sinY
    
    if state in [FLY, TAKEOFF, LAND]:
        pull_mag = math.sqrt(body_ex**2 + body_ey**2)
        if pull_mag > MAX_PULL:
            body_ex = (body_ex / pull_mag) * MAX_PULL
            body_ey = (body_ey / pull_mag) * MAX_PULL
            
        # Tangential slide (going around the obstacle
        sx, sy = 0.0, 0.0
        # Base Repulsion (from obstacles)
        rx, ry = 0.0, 0.0
            
        # Smooth blending scalers (1.0 = full target pull)
        raw_scale_x = 1.0
        raw_scale_y = 1.0
        
        # Obstacle avoidance
        if state == FLY:
            local_avoid_x = 0.0
            local_avoid_y = 0.0
        
            # front sensor
            if 'front' in distance_sensors:
                f_val = distance_sensors['front'].getValue()
            else:
                # Fallback value if nothing detected
                f_val = 3.0
            
            # back sensor
            if 'back' in distance_sensors:
                b_val = distance_sensors['back'].getValue()
            else:
                # Fallback value if nothing detected
                b_val = 3.0
            
            # left sensor
            if 'left' in distance_sensors:
                l_val = distance_sensors['left'].getValue()
            else:
                # Fallback value if nothing detected
                l_val = 3.0
            
            # right sensor
            if 'right' in distance_sensors:
                r_val = distance_sensors['right'].getValue()
            else:
                # Fallback value if nothing detected
                r_val = 3.0
        
            
            # Front
            if f_val < AVOID_THRESHOLD:
                rx -= (AVOID_THRESHOLD - f_val) * AVOID_WEIGHT
            # Back
            if b_val < AVOID_THRESHOLD:
                rx += (AVOID_THRESHOLD - b_val) * AVOID_WEIGHT
            # Left
            if l_val < AVOID_THRESHOLD:
                ry -= (AVOID_THRESHOLD - l_val) * AVOID_WEIGHT
            # Right
            if r_val < AVOID_THRESHOLD:
                ry += (AVOID_THRESHOLD - r_val) * AVOID_WEIGHT      
        
            
            
            # Y axis latch (if front / back path is blocked, pick a side (left / right) and lock it in)
            if f_val < AVOID_THRESHOLD or b_val < AVOID_THRESHOLD:
                closest_front_back = min(f_val, b_val)
                # Linear blend: 0.0 when very close, scales to 1.0 right at the threshold edge
                raw_scale_x = clamp((closest_front_back - MIN_DIST) / (AVOID_THRESHOLD - MIN_DIST), 0.0, 1.0)
                
                # Suppress sideways target tracking, preventing cross axis tug of war
                raw_scale_y = raw_scale_x
                
                # Fresh decision needed
                if bypass_dir_y == 0.0:
                    # More room on the left
                    if l_val > r_val + 0.1:
                        bypass_dir_y = 1.0
                    # More room on the right
                    elif r_val > l_val + 0.1:
                        bypass_dir_y = -1.0
                    # Even wall: slide towards target's relative direction
                    else:
                        if abs(body_ey) > 0.01:
                            bypass_dir_y = math.copysign(1.0, body_ey)
                sy = bypass_dir_y * SLIDE_WEIGHT
            # Path is clear, drop direction latch
            else:
                bypass_dir_y = 0.0
                
            # X axis latch (if left / right path is blocked, pick a side (front / back) and lock it in)
            if l_val < AVOID_THRESHOLD or r_val < AVOID_THRESHOLD:
                closest_left_right = min(l_val, r_val)
                # Linear blend: 0.0 when very close, scales to 1.0 right at the threshold edge
                raw_scale_y = clamp((closest_left_right - MIN_DIST) / AVOID_THRESHOLD - MIN_DIST, 0.0, 1.0)
                
                # Suppress forward / backward target tracking, preventing cross axis tug of war
                raw_scale_x = raw_scale_y
                
                # Fresh decision needed
                if bypass_dir_x == 0.0:
                    # More room in front
                    if f_val > b_val + 0.1:
                        bypass_dir_x = 1.0
                    # More room behind
                    elif b_val > f_val + 0.1:
                        bypass_dir_x = -1.0
                    # Even wall: slide towards target's relative direction
                    else:
                        if abs(body_ex) > 0.01:
                            bypass_dir_x = math.copysign(1.0, body_ex)
                sx = bypass_dir_x * SLIDE_WEIGHT
            # Path is clear, drop direction latch
            else:
                bypass_dir_x = 0.0   
            
            # Asymmetric memory filters for corners
            # Filter all avoidance vectors
            f_rx = filter_force(f_rx, rx)
            f_ry = filter_force(f_ry, ry)
            f_sx = filter_force(f_sx, sx)
            f_sy = filter_force(f_sy, sy)
            
            # Scale decay oppositely (Target scale dropping danger / attack, recovering is safe / release)
            # Smooth out X-axis scaling
            if raw_scale_x < f_target_scale_x:
                f_target_scale_x += (raw_scale_x - f_target_scale_x) * ATTACK
            else:
                f_target_scale_x += (raw_scale_x - f_target_scale_x) * RELEASE
            # Smooth out Y-axis scaling
            if raw_scale_y < f_target_scale_y:
                f_target_scale_y += (raw_scale_y - f_target_scale_y) * ATTACK
            else:
                f_target_scale_y += (raw_scale_y - f_target_scale_y) * RELEASE
            
            # Apply scale smoothly to the original tracking errors before adding avoidance forces
            # inject combined locked vectors back into tracking loop         
            body_ex = (body_ex * f_target_scale_x) + f_rx + f_sx
            body_ey = (body_ey * f_target_scale_y) + f_ry + f_sy
            
            # zero integrals during evasive maneuvers to eliminate memory whiplash
            if state == FLY and (bypass_dir_x != 0.0 or bypass_dir_y != 0.0):
                i_ex = 0.0
                i_ey = 0.0
            else:
                # Update Integrals with clamping to prevent Windup using only target position data
                i_ex = clamp(i_ex + body_ex * dt, -0.5, 0.5)
                i_ey = clamp(i_ey + body_ey * dt, -0.5, 0.5)
                  
        # PID Formula: (Proportional + Integral) - Derivative
        pitch_cmd = (K_POS_P * body_ex) + (K_POS_I * i_ex) - (K_POS_D * body_vx)
        roll_cmd = -((K_POS_P * body_ey) + (K_POS_I * i_ey) - (K_POS_D * body_vy))
        
        desired_pitch = clamp(pitch_cmd + pitch_d, -0.3, 0.3)
        desired_roll = clamp(roll_cmd + roll_d, -0.3, 0.3)
        
        # 6. Altitude Controller
        raw_v_error = targetPos.z - currentPos.z
        
        i_ez = clamp(i_ez + raw_v_error * dt, -2.0, 2.0)
        
        # calculate vertical input
        # mix: Proportional (P) + Integral (I) - Derivative(D) + Offset
        p_term = K_VERTICAL_P * raw_v_error
        i_term = K_VERTICAL_I * i_ez
        d_term = K_VERTICAL_D * f_zVel
        
        # subtract d_term to oppose direction of travel
        vertical_input = p_term + i_term - d_term+ K_VERTICAL_OFFSET
    
        # 7. Attitude Controller with Gyro Damping
        roll_input = K_ROLL_P * clamp(roll - desired_roll, -1.0, 1.0) + (roll_vel * K_GYRO_D)
        pitch_input = K_PITCH_P * clamp(pitch - desired_pitch, -1.0, 1.0) + (pitch_vel * K_GYRO_D)
        
        target_rad = math.radians(targetYaw)
        # Shift 0-2pi to -pi to pi to match IMU
        if target_rad > math.pi: target_rad -= 2.0 * math.pi
        
        yaw_error = target_rad - yaw
        
        # Shortest path wrap-around for rotation
        while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi: yaw_error += 2.0 * math.pi
        
        yaw_input = (K_YAW_P * yaw_error) - (yaw_vel * K_GYRO_D)
        
        # Altitude loss compensation while moving
        # calculate thrust loss due to tilt
        cos_factor = math.cos(roll) * math.cos(pitch)
        
        # "ideal" multiplier for counteracting gravity
        # clamp factor to prevent motor oversaturation (maximum 40% boost)
        # 0.707 corresponss to ~45 degree tilt
        boost_requirement = 1.0 / max(cos_factor, 0.707)
        
        thrustCompensation = 1.0 + (boost_requirement - 1.0) * K_COMP_AGGRESSION
        
        # apply compensation to vertical components only
        base_thrust = (K_VERTICAL_THRUST + vertical_input) * thrustCompensation
        
        # 8. Motor Mixer
        fl_v = base_thrust - roll_input + pitch_input - yaw_input
        fr_v = base_thrust + roll_input + pitch_input + yaw_input
        rl_v = base_thrust - roll_input - pitch_input + yaw_input
        rr_v = base_thrust + roll_input - pitch_input - yaw_input
        
        # 9. Set Velocities (using clamped speed limit)
        fl.setVelocity(clamp(fl_v, 0.0, MOTOR_SPEED_LIMIT))
        fr.setVelocity(clamp(-fr_v, -MOTOR_SPEED_LIMIT, 0.0))
        rl.setVelocity(clamp(-rl_v, -MOTOR_SPEED_LIMIT, 0.0))
        rr.setVelocity(clamp(rr_v, 0.0, MOTOR_SPEED_LIMIT))
        
    else:
        # IDLE or DONE: Keep motors off
        fl.setVelocity(0.0); fr.setVelocity(0.0)
        rl.setVelocity(0.0); rr.setVelocity(0.0)
    
     # ── LAND — staged descent
    if state == LAND:
        if land_stage < len(LAND_SEQUENCE):
            tgt, hold = LAND_SEQUENCE[land_stage]
            targetPos.z = tgt
            
            # Advance only when altitude is reached AND hold time has elapsed
            if abs(currentPos.z - tgt) < ALT_REACHED and (t- land_stage_timer) > hold:
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
