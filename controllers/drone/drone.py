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
                    targetPos = Vector3.from_msg(' '.join(parts[2:5]))
                    print(f"[COMMS] New target received: X:{targetPos.x:.2f} Y:{targetPos.y:.2f} Z:{targetPos.z:.2f}")
                except:
                    print(f"[COMMS] Error parsing coordinate data.")
            elif len(parts) >= 3 and parts[0] == drone_name and parts[1] == 'PATH':
                try:
                    wps_start = 2
                    yaw_override = None
                    if parts[2].startswith('YAW:'):
                        yaw_override = float(parts[2][4:])
                        wps_start = 3
                    waypoints = [Vector3(*map(float, wp.split(','))) for wp in parts[wps_start:]]
                    new_path = Path(waypoints)
                    if current_path is None and not path_queue:
                        current_path = new_path
                        path_index = 0
                        if yaw_override is not None:
                            targetYaw = yaw_override
                        # Only chase the first waypoint immediately if already flying.
                        # If still in TAKEOFF, leave targetPos alone so the drone reaches
                        # TAKEOFF_ALT first — the TAKEOFF completion block sets targetPos.
                        if len(current_path) > 0 and state == FLY:
                            targetPos = current_path[0]
                        print(f"[COMMS] Path started ({state}): {len(current_path)} waypoints")
                    else:
                        path_queue.append((new_path, yaw_override))
                        print(f"[COMMS] Path queued: {len(new_path)} waypoints (queue depth={len(path_queue)})")
                except Exception as e:
                    print(f"[COMMS] Error parsing PATH: {e}")
            elif len(parts) == 2 and parts[0] == drone_name and parts[1] == 'TAKEOFF':
                if state == IDLE:
                    targetPos.x = currentPos.x
                    targetPos.y = currentPos.y
                    targetPos.z = TAKEOFF_ALT
                    targetYaw = yaw
                    state = TAKEOFF
                    print(f'>> [COMMS] TAKEOFF command received')
            elif len(parts) == 2 and parts[0] == drone_name and parts[1] == 'LAND':
                if state == FLY:
                    # Clear any pending work and return to the spawn position before landing.
                    current_path = None
                    path_queue.clear()
                    task_queue.clear()
                    current_task = None
                    targetPos = Vector3(home_pos.x, home_pos.y, TAKEOFF_ALT)
                    state = RETURN
                    print(f'>> [COMMS] LAND command received — returning to home ({home_pos.x:.2f},{home_pos.y:.2f})')
            elif len(parts) == 8 and parts[0] == drone_name and parts[1] == 'TASK' and parts[2] == 'PICKUP':
                try:
                    brick_id = int(parts[3])
                    px, py, pz = float(parts[4]), float(parts[5]), float(parts[6])
                    pickup_yaw = float(parts[7])
                    task_queue.append(('PICKUP', brick_id, Vector3(px, py, pz), pickup_yaw))
                    print(f'[TASK] Queued PICKUP brick {brick_id} → ({px:.2f},{py:.2f},{pz:.2f}m) yaw={pickup_yaw:.1f}°')
                except (ValueError, IndexError):
                    print(f'[TASK] Malformed TASK PICKUP: {packet}')
            elif len(parts) == 8 and parts[0] == drone_name and parts[1] == 'TASK' and parts[2] == 'PLACE':
                try:
                    brick_id = int(parts[3])
                    tx, ty, tz, rot = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                    place_fly_z = max(tz + PLACE_ALT_OFFSET, PICKUP_ALT)
                    task_queue.append(('PLACE', brick_id, Vector3(tx, ty, place_fly_z), tx, ty, tz, rot))
                    print(f'[TASK] Queued PLACE brick {brick_id} → fly to z={place_fly_z:.2f}m, place at z={tz:.3f}')
                except (ValueError, IndexError):
                    print(f'[TASK] Malformed TASK PLACE: {packet}')
            receiver.nextPacket()
    # send drone's current status (current and target location) at twice the speed of the print counter
    if emitter and print_counter % 50 == 0:
        status_msg = f"{drone_name} CURRENTPOS {currentPos.to_msg()} TARGETPOS {targetPos.to_msg()}"
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
        
    # 4. State Machine
    # IDLE → TAKEOFF on command; TAKEOFF → FLY once cruise altitude is reached;
    # FLY is the normal operating state; RETURN flies back to home XY then hands off
    # to LAND; LAND triggers staged descent; DONE cuts motors.
    print_counter += 1
    if state == RETURN:
        xy_dist = math.sqrt((currentPos.x - home_pos.x) ** 2 + (currentPos.y - home_pos.y) ** 2)
        if xy_dist < TASK_ARRIVAL_THRESHOLD:
            state = LAND
            land_stage = 0
            land_stage_timer = t
            print(f'>> Arrived at home position, beginning auto-land')

    if state == TAKEOFF and abs(currentPos.z - TAKEOFF_ALT) < ALT_REACHED:
        state = FLY
        if current_path is not None and path_index < len(current_path):
            targetPos = current_path[path_index]
            print(f'>> TAKEOFF COMPLETE. Resuming path at waypoint {path_index}: ({targetPos.x:.2f},{targetPos.y:.2f},{targetPos.z:.2f})')
        else:
            targetPos.x = currentPos.x
            targetPos.y = currentPos.y
            print(f'>> TAKEOFF COMPLETE. Holding position at X:{targetPos.x:.2f}, Y:{targetPos.y:.2f}. Switching to fly state')
    
    if state == FLY and print_counter % 100 == 0:
        # Periodic status update
        actual_yaw_deg = math.degrees(yaw) % 360.0
        #print(f'Target: (X: {targetPos.x:.2f}, Y: {targetPos.y:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {targetPos.z:.2f})\nActual: (X: {currentPos.x:.2f}, Y: {currentPos.y:.2f}) | Yaw: {actual_yaw_deg:.2f}° | Altitude: {currentPos.z:.2f}')

    # Advance through path waypoints when the current one is reached.
    # Each brick task uses three path legs sent sequentially:
    #   1. Ascend to cruise altitude directly above current position
    #   2. Cruise horizontally to above the target (pile or placement site)
    #   3. Descend to pickup/place altitude
    # Separating ascend and cruise prevents drones from cutting diagonally through
    # the pile or the partially-built structure at low altitude.
    if state == FLY and current_path is not None and path_index < len(current_path):
        wp = current_path[path_index]
        dist_3d = Vector3.distance(currentPos, wp)
        #if print_counter % 100 == 0:
            #print(f'[PATH] idx={path_index}/{len(current_path)-1} | dist_3d={dist_3d:.3f} | threshold={WAYPOINT_THRESHOLD}')
            #print(f'[PATH] pos=({currentPos.x:.2f},{currentPos.y:.2f},{currentPos.z:.2f}) | wp=({wp.x:.2f},{wp.y:.2f},{wp.z:.2f})')
        if dist_3d < WAYPOINT_THRESHOLD:
            path_index += 1
            if path_index < len(current_path):
                targetPos = current_path[path_index]
                print(f'[{drone_name}] [PATH] Waypoint {path_index} = ({targetPos.x:.2f},{targetPos.y:.2f},{targetPos.z:.2f})')
            else:
                # Path complete — start next queued path, or activate task
                current_path = None
                if path_queue:
                    next_path, next_yaw = path_queue.pop(0)
                    current_path = next_path
                    path_index = 0
                    if next_yaw is not None:
                        targetYaw = next_yaw
                    if len(current_path) > 0:
                        targetPos = current_path[0]
                    yaw_info = f' yaw→{next_yaw:.1f}°' if next_yaw is not None else ''
                    print(f'[PATH] Next leg: {len(current_path)} waypoints{yaw_info}')
                elif task_queue and current_task is None:
                    current_task = task_queue.pop(0)
                    dest = current_task[2]
                    if current_task[0] == 'PICKUP':
                        targetYaw = current_task[3]
                    elif current_task[0] == 'PLACE':
                        targetYaw = current_task[6]
                    targetPos = Vector3(dest.x, dest.y, dest.z)
                    print(f'[TASK] All paths done, activating {current_task[0]} brick {current_task[1]} → ({targetPos.x:.2f},{targetPos.y:.2f},{targetPos.z:.2f})')

    # Fallback: if idle with no current path, advance the queue
    if state == FLY and current_task is None and current_path is None:
        if path_queue:
            next_path, next_yaw = path_queue.pop(0)
            current_path = next_path
            path_index = 0
            if next_yaw is not None:
                targetYaw = next_yaw
            if len(current_path) > 0:
                targetPos = current_path[0]
            print(f'[PATH] Fallback: starting queued path, {len(current_path)} waypoints')
        elif task_queue:
            current_task = task_queue.pop(0)
            dest = current_task[2]
            if current_task[0] == 'PICKUP':
                targetYaw = current_task[3]
            elif current_task[0] == 'PLACE':
                targetYaw = current_task[6]
            targetPos = Vector3(dest.x, dest.y, dest.z)
            print(f'[TASK] Activating {current_task[0]} brick {current_task[1]} → ({dest.x:.2f},{dest.y:.2f},{dest.z:.2f}) (fallback)')

    # Emit PICKUP / PLACE signal when task descent position is reached
    if state == FLY and current_task is not None:
        if Vector3.distance(currentPos, current_task[2]) < TASK_ARRIVAL_THRESHOLD:
            verb = current_task[0]
            brick_id = current_task[1]
            if verb == 'PICKUP':
                emitter.send(f"{drone_name} PICKUP {brick_id}".encode('utf-8'))
                print(f'[TASK] Emitted PICKUP brick {brick_id}')
            elif verb == 'PLACE':
                _, _, _, tx, ty, tz, rot = current_task
                emitter.send(f"{drone_name} PLACE {brick_id} {tx:.4f} {ty:.4f} {tz:.4f} {rot:.4f}".encode('utf-8'))
                print(f'[TASK] Emitted PLACE brick {brick_id} at ({tx:.2f},{ty:.2f},{tz:.2f})')
            current_task = None

    # 5. Position Controller (World to Body)
    cosY, sinY = math.cos(yaw), math.sin(yaw)
    ex = (targetPos.x - currentPos.x)
    ey = (targetPos.y - currentPos.y)
    
    body_vx = f_xVel * cosY + f_yVel * sinY
    body_vy = f_yVel * cosY - f_xVel * sinY
    
    # Calculate TRUE raw errors in drone body frame
    body_ex = ex * cosY + ey * sinY
    body_ey = ey * cosY - ex * sinY
    
    if state in [FLY, TAKEOFF, LAND, RETURN]:
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
        if state == FLY and OBSTACLE_AVOIDANCE:
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
                raw_scale_y = clamp((closest_left_right - MIN_DIST) / (AVOID_THRESHOLD - MIN_DIST), 0.0, 1.0)
                
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
        
        # Thrust compensation for altitude loss during horizontal movement.
        # When tilted the vertical component of thrust = thrust × cos(roll) × cos(pitch).
        # Boosting by the inverse recovers lost lift. The 0.707 floor caps boost at 40 %
        # (≈45° tilt) to avoid motor saturation on aggressive manoeuvres.
        cos_factor = math.cos(roll) * math.cos(pitch)
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
