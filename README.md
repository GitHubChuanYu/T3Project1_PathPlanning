# CarND-Path-Planning-Project
This is Chuan's writeup report for Udacity self-driving car nano degree program term 3 project 1 Path Planning

---
[//]: # (Image References)

[image1]: ./BehaviorPlanning.png "BehaviorPlanning"
[image2]: ./visual_aid.png "visual_aid"

## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The self driving car's localization and sensor fusion data will be provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Main data provided from the Simulator to the C++ Program

### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Code implementation and explanation

The path planning alogrithm in main.cpp mainly contains two part:

* The first part is **behavior planning** which is to plan the next step behavior of the car, such as keep lane (accelerate), keep lane (decelerate), change lane left, and change lane right according to surrounding traffic environment.

* The second part is **trajectory generation** which is responsible for generating a smooth trajectory for self-driving car to follow based on the behavioral command from **behavior planning**.

### Behavior planning

In this project, the **behavior planning** part is a simple state transition logic to decide the transition between below four states:

* State 1: Keep lane and accelerate to target vehicle speed
* State 2: Keep lane and decelerate to follow the front vehicle
* State 3: Change to right lane
* State 4: Change to left lane

The transition among these four states can be decribed as follows:
  ![alt text][image1]
  
To check whether whether it is possible to do lane change, we need to use sensor fusion data to predict the future position of vehicle in the target lane and compare the **s** position of it with future **s** position (in Frenet coordinate) of self-driving car to decide whether we can do lane change or not.

All the behavior planning code is between line 128 and line 204 in [main.cpp](https://github.com/GitHubChuanYu/T3Project1_PathPlanning/blob/master/src/main.cpp) file.

### Trajectory generation

To ensure smooth trajectory generation, the **spline** method is used to fit target way points. That is why [spline.h](https://github.com/GitHubChuanYu/T3Project1_PathPlanning/blob/master/src/spline.h) is included.

As suggested in [Project Q&A](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d), five way points are picked for spline fitting. The first two points are from previous path (or if previous path is almost empty, then use current and previous car location x, y). The rest three way points are transformed from three waypoints defined in Frenet coordinate with target **s** of 30m, 60m, and 90m away from current self-driving car **s** and target **d** in target lane calcuated from **behavior planning** part. The code is between line 233 and line 294 in [main.cpp](https://github.com/GitHubChuanYu/T3Project1_PathPlanning/blob/master/src/main.cpp) file:

* Note: During spline fitting, all the waypoints are transformed from global coordinate system to car's coordinate system so that they are easier for calculating. The detailed theory for this transform can be found in this [link](https://www.miniphysics.com/coordinate-transformation-under-rotation.html).

After spline trajectory is calculated. We can calculate the final waypoints for path planner:

* All the previous path waypoints are included in current path waypoints to ensure smooth path to avoid large value of acceleration and jerk.

* The total waypoints are 50, so the rest of waypoints (50 - previous path waypoints) are generated from fitted spline curve:

  * First to ensure the planned waypoints along spline will not have large acceleration and jerk, a method is introduced in [Project Q&A](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) as shown in this figure:

  ![alt text][image2]
  
  * Then rest of waypoints calculated from fitted spline are tranformed back from car's coordinate system to global (map's) coordinate system.
  
The code for overall smooth trajectory generation is between line 300 and line 335 in [main.cpp](https://github.com/GitHubChuanYu/T3Project1_PathPlanning/blob/master/src/main.cpp) file. 
  
