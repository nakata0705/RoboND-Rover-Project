import numpy as np
import cv2

def decision_roverstable(Rover):
    acceptable_error_angle = 0.4
    if (Rover.pitch > 360 - acceptable_error_angle or Rover.pitch < acceptable_error_angle) and (Rover.roll > 360 - acceptable_error_angle or Rover.roll < acceptable_error_angle):
        return True
    return False

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh_low=(160, 160, 160), rgb_thresh_high=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    r_thresh = (img[:,:,0] > rgb_thresh_low[0]) & (img[:,:,0] <= rgb_thresh_high[0])
    g_thresh = (img[:,:,1] > rgb_thresh_low[1]) & (img[:,:,1] <= rgb_thresh_high[1])
    b_thresh = (img[:,:,2] > rgb_thresh_low[2]) & (img[:,:,2] <= rgb_thresh_high[2])
    above_thresh = r_thresh & g_thresh & b_thresh
    
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

def invtranslate_pix(xpix_world, ypix_world, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_world - xpos) * scale
    ypix_translated = (ypix_world - ypos) * scale
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    
    # Record origin
    if Rover.origin == None:
        Rover.origin = Rover.pos

    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, src, dst)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable = color_thresh(warped, (160, 160, 160), (255, 255, 255))
    rocksample = color_thresh(warped, (120, 120, 1), (255, 255, 80))
    obstacle = color_thresh(warped, (1, 1, 1), (159, 159, 159))
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle * 255
    Rover.vision_image[:,:,1] = rocksample * 255
    Rover.vision_image[:,:,2] = navigable * 255
    
    # 5) Convert map image pixel values to rover-centric coords
    obstacle_rovercoords = rover_coords(obstacle);
    rocksample_rovercoords = rover_coords(rocksample);
    navigable_rovercoords = rover_coords(navigable);
            
    # 6) Convert rover-centric pixel values to world coordinates
    obstacle_worldcoords = pix_to_world(obstacle_rovercoords[0], obstacle_rovercoords[1], Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    rocksample_worldcoords = pix_to_world(rocksample_rovercoords[0], rocksample_rovercoords[1], Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    navigable_worldcoords = pix_to_world(navigable_rovercoords[0], navigable_rovercoords[1], Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if decision_roverstable(Rover) == True:
        Rover.worldmap[obstacle_worldcoords[1], obstacle_worldcoords[0], 0] += 1
        Rover.worldmap[rocksample_worldcoords[1], rocksample_worldcoords[0], 1] += 2
        Rover.worldmap[navigable_worldcoords[1], navigable_worldcoords[0], 2] += 2
        Rover.worldmap[navigable_worldcoords[1], navigable_worldcoords[0], 0] = 0

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_rovercoords[0], navigable_rovercoords[1])
    Rover.nav_rock_dists, Rover.nav_rock_angles = to_polar_coords(rocksample_rovercoords[0], rocksample_rovercoords[1])
    
    # Set origin in Rover coordinate system
    origin_translated = invtranslate_pix(Rover.origin[0], Rover.origin[1], Rover.pos[0], Rover.pos[1], 10)
    origin_rovercoord = rotate_pix(origin_translated[0], origin_translated[1], -Rover.yaw)
    Rover.origin_polarcoord = to_polar_coords(origin_rovercoord[0], origin_rovercoord[1])
    
    return Rover