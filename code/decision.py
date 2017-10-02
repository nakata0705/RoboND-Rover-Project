import numpy as np

# Decide if Rover is stopped or not
def decision_roverstopped(Rover, consideranglevelocity = True):
    acceptable_error_velocity = 0.1
    acceptable_error_anglevelocity = 0.1
    if np.absolute(np.sum(Rover.vel_history)) <= acceptable_error_velocity and \
       (consideranglevelocity == False or \
        (consideranglevelocity == True and \
         np.absolute(np.sum(Rover.yawvel_history)) <= acceptable_error_anglevelocity and \
         np.absolute(np.sum(Rover.pitchvel_history)) <= acceptable_error_anglevelocity and \
         np.absolute(np.sum(Rover.rollvel_history)) <= acceptable_error_anglevelocity)):
        print ("decision_roverstopped: True\n")
        return True
    print ("decision_roverstopped: False\n")
    return False

# Decide if Rover can go forward or not
def decision_cangoforward(Rover, min_forward_dist = 10.0):
    min_side_dist = 5.0
    if len(Rover.nav_angles) >= Rover.stop_forward and Rover.dist_front >= min_forward_dist:
        print ("decision_cangoforward: True\n")
        return True
    print ("decision_cangoforward: False\n")
    return False

# Decide if Rover is stuck
def decision_stuck(Rover):
    return Rover.total_time - Rover.start_forwardmode_time > 1 and decision_roverstopped(Rover, True)    

def steering_gostraight(Rover):
    print("Go straight mode: Trying to find a wall")
    return 0

def steering_keepleft(Rover):
    coefficient_base = (1.0 / Rover.max_vel)
    coefficient_p = coefficient_base
    coefficient_d = coefficient_base
    Rover.steermode = "Left"
    print("Keep left mode: Steering angle factors P = %f D = %f\n" % ((Rover.dist_left - Rover.target_dist_side) * coefficient_p, (Rover.dist_left - Rover.last_dist_left) * coefficient_d))
    return np.clip((Rover.dist_left - Rover.target_dist_side) * coefficient_p + (Rover.dist_left - Rover.last_dist_left) * coefficient_d, -15, 15)

def steering_keepright(Rover):
    coefficient_base = (1.0 / Rover.max_vel)
    coefficient_p = coefficient_base
    coefficient_d = coefficient_base
    Rover.steermode = "Right"
    print("Keep right mode: Steering angle factors P = %f D = %f\n" % ((Rover.dist_right - Rover.target_dist_side) * coefficient_p, (Rover.dist_right - Rover.last_dist_left) * coefficient_d))
    return -np.clip((Rover.dist_right - Rover.target_dist_side) * coefficient_p + (Rover.dist_right - Rover.last_dist_left) * coefficient_d, -15, 15)

def steering_keepmiddle(Rover):
    coefficient_base = (1.0 / Rover.max_vel)
    coefficient_p = coefficient_base
    coefficient_d = coefficient_base
    print("Keep middle mode: Steering angle factors P = %f D = %f\n" % ((Rover.dist_left - Rover.dist_right) * coefficient_p, ((Rover.dist_left - Rover.dist_right) - (Rover.last_dist_left - Rover.last_dist_right)) * coefficient_d))
    return  np.clip((Rover.dist_left - Rover.dist_right) * coefficient_p + ((Rover.dist_left - Rover.dist_right) - (Rover.last_dist_left - Rover.last_dist_right)) * coefficient_d, -15, 15)

# Decide the steering angle
def decision_steerangle(Rover):
    #std = np.std(Rover.nav_angles * 180/np.pi)
    #return np.clip(np.mean(Rover.nav_angles * 180/np.pi) + std/2, -10, 10)
    steering = 0;
    
    if Rover.seekorigin == True:
        # Looking for origin and close to origin
        print("seek origin")
        if Rover.dist_left >= Rover.target_dist_side and Rover.dist_right >= Rover.target_dist_side:
            steering = np.clip(Rover.origin_polarcoord[1] * 180/np.pi, -15, 15)
        elif Rover.dist_left < Rover.target_dist_side and Rover.dist_right < Rover.target_dist_side:
            steering = steering_keepmiddle(Rover)
        elif Rover.dist_left < Rover.target_dist_side:
            steering = steering_keepleft(Rover)
        else:
            steering = steering_keepright(Rover)
    elif Rover.nav_rock_angles is not None and len(Rover.nav_rock_angles) > 0:
        # Rover found the rock. Immediately go toward the rock
        steering = np.clip(np.mean(Rover.nav_rock_angles * 180/np.pi) - 5, -15, 15)
    elif Rover.dist_left > Rover.target_dist_side and Rover.dist_right > Rover.target_dist_side:
        # The distance to the left wall and to the right wall are far
        if Rover.dist_left > Rover.target_dist_side * 4 and Rover.last_dist_left > Rover.target_dist_side * 4 and Rover.last_dist_left < Rover.dist_left:
            steering_gostraight(Rover)
        else:
            steering = steering_keepleft(Rover)
    else:
        steering = steering_keepmiddle(Rover)
    return steering

# Decide the throttle amount
def decision_throttle(Rover):
    if Rover.nav_rock_angles is not None and len(Rover.nav_rock_angles) > 0:
        # Trying to pick up the rock. Slow down.
        Rover.max_vel = 1.0
    elif Rover.dist_left > Rover.dist_left * 2:
        # Lost left wall. Slow down.
        Rover.max_vel = 0.2
    elif Rover.seekorigin == True and Rover.origin_polarcoord[0] < 10:
        # A wall in front. Slow down.
        Rover.max_vel = 0.2
    elif Rover.seekorigin == True and Rover.origin_polarcoord[0] < 20:
        # A wall in front. Slow down.
        Rover.max_vel = 0.5
    elif Rover.dist_front < 40 or (Rover.seekorigin == True and Rover.origin_polarcoord[0] < 40):
        # A wall in front. Slow down.
        Rover.max_vel = 1.0
    elif Rover.dist_front < 80 or (Rover.seekorigin == True and Rover.origin_polarcoord[0] < 80):
        # A wall in front but far. Medium speed.
        Rover.max_vel = 2.0
    else:
        # No wall in front. High speed..
        Rover.max_vel = 3.0
    
    # Control speed based on the max speed
    if np.absolute(Rover.vel) < Rover.max_vel:
        Rover.throttle = Rover.throttle_set
        Rover.brake = 0
    else:
        Rover.throttle = 0
        Rover.brake = 0.2
    
    
# Decide how far Rover is from walls.
def decision_dist(Rover):
    index_left = [];
    index_front = [];
    index_right = [];
    
    for index, val in enumerate(Rover.nav_angles):
        if val < -np.pi/4 + np.pi/24:
            index_right.append(index)
        elif (val > -np.pi/(24*2) and val <= 0) or (val < np.pi/(24*2) and val >= 0):
            index_front.append(index)
        elif val > np.pi/4 - np.pi/24:
            index_left.append(index)
    
    if len(index_left) > 0:
        dist_left = Rover.nav_dists[index_left]
        Rover.dist_left = np.max(dist_left)
    else:
        Rover.dist_left = 0

    if len(index_front) > 0:
        dist_front = Rover.nav_dists[index_front]
        Rover.dist_front = np.max(dist_front)
    else:
        Rover.dist_front = 0
    
    if len(index_right) > 0:
        dist_right = Rover.nav_dists[index_right]
        Rover.dist_right = np.max(dist_right)
    else:
        Rover.dist_right = 0
    
    # Copy the values as a past value if there is no history.
    if Rover.last_dist_front == None:
        Rover.last_dist_front = Rover.dist_front
    if Rover.last_dist_left == None:
        Rover.last_dist_left = Rover.dist_left
    if Rover.last_dist_right == None:
        Rover.last_dist_right = Rover.dist_right

def action_forward(Rover):
    decision_throttle(Rover)
    Rover.steer = decision_steerangle(Rover)
    Rover.start_forwardmode_time = Rover.total_time
    Rover.mode = 'forward'
        
def action_stop(Rover):
    # Set mode to "stop" and hit the brakes!
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'

def action_end(Rover):
    # Set mode to "stop" and hit the brakes!
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'end'
    
def action_stuckrecovery(Rover):
    action_stop(Rover)
    Rover.start_stuckmode_time = Rover.total_time
    Rover.mode = 'stuckrecovery'                    

def action_stuckrecovery2(Rover):
    action_stop(Rover)
    Rover.start_stuckmode_time = Rover.total_time
    Rover.mode = 'stuckrecovery2'                    
    
def update_velocity_history(Rover):
    # Update veolcity history
    Rover.vel_history.append(Rover.vel)
    Rover.vel_history.pop(0);
    
    if Rover.last_yaw == None:
        Rover.last_yaw = Rover.yaw
    if Rover.last_pitch == None:
        Rover.last_pitch = Rover.pitch
    if Rover.last_roll == None:
        Rover.last_roll = Rover.roll
    
    Rover.yawvel_history.append(Rover.yaw - Rover.last_yaw)
    Rover.yawvel_history.pop(0)
    Rover.pitchvel_history.append(Rover.pitch - Rover.last_pitch)
    Rover.pitchvel_history.pop(0)
    Rover.rollvel_history.append(Rover.roll - Rover.last_roll)
    Rover.rollvel_history.pop(0)

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # Update velocity history
    update_velocity_history(Rover)
    
    # Update distance from walls
    decision_dist(Rover)
          
    print ("\nLeft (%f) Front (%f) Right (%f)" % (Rover.dist_left, Rover.dist_front, Rover.dist_right))
    #print ("\nLeft (%f) Front (%f) Right (%f)\n" % (Rover.last_dist_left, Rover.last_dist_front, Rover.last_dist_right))
    
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            print("Rover FORWARD")
            # Check the extent of navigable terrain
            if Rover.near_sample == True:
                action_stop(Rover)
            elif Rover.nav_rock_angles is not None and len(Rover.nav_rock_angles) > 0:
                # Detect stuck
                if decision_stuck(Rover):
                    action_stuckrecovery(Rover)
                else:
                    # If mode is forward, navigable terrain looks good 
                    decision_throttle(Rover)
                    Rover.steer = decision_steerangle(Rover)
            elif Rover.seekorigin == True and Rover.origin_polarcoord[0] < 20:
                action_end(Rover)
            elif decision_cangoforward(Rover) == True:
                # Detect stuck
                if decision_stuck(Rover):
                    action_stuckrecovery(Rover)
                else:
                    # If mode is forward, navigable terrain looks good 
                    decision_throttle(Rover)
                    Rover.steer = decision_steerangle(Rover)
            else:
                action_stop(Rover)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print("Rover STOP")
            # If we're in stop mode but still moving keep braking
            if decision_roverstopped(Rover, False) == False:
                action_stop(Rover)
            # If we're not moving (vel < 0.2) then do something else
            #elif Rover.vel <= 0.2:
            else:
                # Now we're stopped and we have vision data to see if there's a path forward
                #if len(Rover.nav_angles) < Rover.go_forward:
                if Rover.near_sample == True:
                    Rover.pick_up = 1
                elif decision_cangoforward(Rover, 20) == False:
                    action_stuckrecovery2(Rover)
                else:
                    action_forward(Rover)
        elif Rover.mode == 'stuckrecovery':
            print("Rover STUCKRECOVERY BACKING UP")
            if (Rover.total_time - Rover.start_stuckmode_time) < 2:
                Rover.brake = 0
                Rover.throttle = -Rover.throttle_set
                Rover.steer = 0
            elif decision_roverstopped(Rover, False) == False:
                action_stop(Rover)
                Rover.mode = 'stuckrecovery'
            else:
                action_stuckrecovery2(Rover)
        elif Rover.mode == 'stuckrecovery2':
            print("Rover STUCKRECOVERY ROTATING")
            # If we're in stop mode but still moving keep braking
            if (Rover.total_time - Rover.start_stuckmode_time) < 1:
                Rover.brake = 0
                Rover.throttle = 0
                if Rover.steermode == "Right":
                    Rover.steer = 15
                else:
                    Rover.steer = -15
            else:
                action_stop(Rover)
        elif Rover.mode == 'end':
            print("Rover END")
            action_end(Rover)
                
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    # Check if Rover should return to home
    if Rover.samples_collected >= Rover.samples_to_find:
        Rover.seekorigin = True        
    
    # Update history
    Rover.last_yaw = Rover.yaw
    Rover.last_pitch = Rover.pitch
    Rover.last_roll = Rover.roll
    
    Rover.last_dist_front = Rover.dist_front
    Rover.last_dist_left = Rover.dist_left
    Rover.last_dist_right = Rover.dist_right
    
    return Rover

