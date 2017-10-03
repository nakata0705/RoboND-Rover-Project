## Project: Search and Sample Return

---

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Describe in your writeup (and identify where in your code) how you modified or added functions to add obstacle and rock sample identification.

1. Modified `color_thresh`  so that the Rover can detect the color between the low and high threshhold.
2. Used following code to detect navigable, rocksample and obstacle vector in the image.
`navigable = color_thresh(warped, (160, 160, 160), (255, 255, 255))
rocksample = color_thresh(warped, (120, 120, 1), (255, 255, 80))
obstacle = color_thresh(warped, (1, 1, 1), (159, 159, 159))`
3. Then convert the vectors in the image coordinate system to another vectors in the rover local coordinate system.
`obstacle_rovercoords = rover_coords(obstacle);
rocksample_rovercoords = rover_coords(rocksample);
navigable_rovercoords = rover_coords(navigable);`
4. For rock and navigable area information, convert them to the rover local polar coordinate for later use in `decision.py`.
`Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_rovercoords[0], navigable_rovercoords[1])
Rover.nav_rock_dists, Rover.nav_rock_angles = to_polar_coords(rocksample_rovercoords[0], rocksample_rovercoords[1])`
5. The obstacle detection is done by `decision_dist` function in `decision.py`, which measures the distance to obstacles/walls of left, front and right of the rover. The function also records the distance information in the last frame to use it for the steering control of the rover.
6. The rock detecton is done by `decision_step` function. The rover simply count the number of vector in `Rover.nav_rock_angles` and call `decision_steerangle` function to steer to the `np.clip(np.mean(Rover.nav_rock_angles * 180/np.pi) - 5, -15, 15)` degree. This angle is used not to go into the wall on left side of the rover.

#### 1. Describe in your writeup how you modified the process_image() to demonstrate your analysis and how you created a worldmap. Include your video output with your submission.

1. Created navigable, rocksample and obstacle vectors in the rover local coordinate system as the code below `obstacle_rovercoords = rover_coords(obstacle);
rocksample_rovercoords = rover_coords(rocksample);
navigable_rovercoords = rover_coords(navigable);`
2. Converted the vectors in the rover local coordinate system to the vector in the world coordinate system as below. `pix_to_world` function uses translation and rotation of the vector for the conversion.
`obstacle_worldcoords = pix_to_world(obstacle_rovercoords[0], obstacle_rovercoords[1], Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    rocksample_worldcoords = pix_to_world(rocksample_rovercoords[0], rocksample_rovercoords[1], Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    navigable_worldcoords = pix_to_world(navigable_rovercoords[0], navigable_rovercoords[1], Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)`
3. Then add color value to `Rover.worldmap` as below. I simply added all image values to world map when the rover's pitch and roll are close to 0.0 (`decision_roverstable` function in `perception.py`) and even added pixels far from the rover even if they are not so reliable.
`if decision_roverstable(Rover) == True:
    Rover.worldmap[obstacle_worldcoords[1], obstacle_worldcoords[0], 0] += 1
    Rover.worldmap[rocksample_worldcoords[1], rocksample_worldcoords[0], 1] += 2
    Rover.worldmap[navigable_worldcoords[1], navigable_worldcoords[0], 2] += 2
    Rover.worldmap[navigable_worldcoords[1], navigable_worldcoords[0], 0] = 0`

The rover video is below.

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/yx9Vl5tLncs/0.jpg)](http://www.youtube.com/watch?v=yx9Vl5tLncs)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### perception.py
`decision_roverstable` function decides whether the rover is close to 0 pitch and roll. This function is used before adding values to the world map.

`perception_step` function modifications those are not yet described in this writeup is that it converts the origin point in world coordinate to the rover local polar coordinate.

##### decosion.py
Various `decision_***` functions decides if the rover is stopped, if the rover can go forward, and if the rover is stuck. To detect rover stuck, the rover uses how long is it in "Forward" mode. Those functions uses the distance to wall information measured in `decision_dist` function. `decision_throttle` and `decision_steerangle` are particularly important two decision functions those controls the rover based on the distance information and the information from other `decision_***` functions. 

`decision_steerangle`  has three modes. One is the exploration mode which uses left hand method, one is steer the rover to go forward a rock, and the last is steer the rover to the origin of the exploration. The steering angle is determined based on these rules.
- the distance and the differential of the distance from left wall and the velocity of the rover (`steering_keepleft` function used in the exploration mode)
- the distance and the differential of the distance from the right wall and the velocity of the rover (`steering_keepright` function used in the origin return mode)
- the distance and the differential of the distance from the both wall and the velocity of the rover (`steering_keepmiddle` function used when going a narrow path in any mode) or 
- the angle to the origin point (The if block which uses `Rover.seekprigin` in `decision_steerangle` function which becomes active after collecting all rocks. Once origin return mode is activated, the rover primarily steer toward the origin point and use either right hand method or left and method when it finds obstacles)

`decision_throttle` switches the maximum speed based on the mode, distance from the wall, the rocks, or the origin point. It also slow down if the rover lost its wall on left.

`decision_step` defines stuckrecovery mode. Once rover detects it's not moving for 1 sec even in "forward" mode, it go back for 2 secs and stop, then turn right or left (based on which hand method the rover is using) for 1 sec.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

I used 640 x 480 resotion with good graphic setting. The FPS is shown in the video. It was about 27 to 30 fps.

- The rover picked up all rocks in 940 sec. The mapped area was 93.9% and Fideliy is 76.1%.
- I applied simple PI control to the steering angle control. It let the rover run. But I feel it is not stable enough. The parameter tuning was not well done.
- I didn't implement the algorithm to avoid already mapped area. As a result, the rover explored the map twice after it found the rock on near the wall on right side of the rover and followed the wall with left hand method. The same happened for the last rock and it left a part of map unexplored.
- I used break and quick steering a lot and it reduced the fidelity.

If I would develop further, I will implement like this.
- First, I will change the control method from the distance from the wall to the direction and the distance from the "target point" and at which direction it should look at the point. The target point is any of rock, the origin and the streak of points with certain distance from the left or right side wall of the rover.
- Based on the direction and angle to the target point, the rover should select the optimal way to steer and accelerate.
- The target point system will be a part of the target point queue system. The rover can queue multiple target points and effectively navigate the world. For example, when the rover finds the rock on far right while it navigating left hand method, it queues the rock position then queue the rover's current position. In that way, the rover can go back to the left hand method safely after picking up the rock.

This was a fun homework!