# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


### Simulator
The simulator can be downloaded from the link provided below. (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


[//]: # (Image References)
[image1]: ./examples/result.png "result"


### Goals
In this project, the goal is to safely navigate a car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We're provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Implementation

The implementation of the project is listed as the following steps:

####  1. Vehicle Class
The vehicle class handles the sensor fusion and localization data. You can find it in "vehicle.h". The function `void Vehicle::CarsAround(vector< vector<double> > sensor_fusion, int prev_size)` would check all the possible cars around ego car. It stores the information of possible cars to a vector `cars`. The function `car Vehicle::NearestCarAheadInLane(int lane, double car_s)` would return the nearest car ahead in a specified lane. The function `bool Vehicle::SafeDistance(double car_s, int lane, double ahead, double behind)` is to check the safety distance between the ego car and the car ahead. If the safe distance is returning true, the ego car would find a safe lane to change and then perform a proper lane change when a slow car is ahead of the ego car.

####  2. Prediction
This part of the code is to utilize the telemetry and sensor fusion data. It helps the ego car to understand the nearby environment. In order to avoid collide with other cars, the ego car needs to know:

a) Whether there is a car in front of us and getting too close. ***(line 324 to line 338)***

b) If the above is true and the car ahead of us is running slow, do we need to be ready to change lane?  ***(line 368 to line 371)***


#### 3. Behavior
This part of the code decides what to do next:
a) If there is a car ahead of the ego car, do we need to change lanes?  ***(line 304 to line 318)***


b) If it's not safe to change lane, do we need to slow down or speed up the ego car? ***(line 340 to line 365)***

This part of the code controls the ego car what to do based on the prediction. Generally, we want to drive the ego car about 49.5 MPH (slightly below the speed limit 50MPH). At the same time, the ego car constantly calls `car Vehicle::NearestCarAheadInLane(int lane, double car_s)` to see if there is a car within 30 meters ahead. If such a car exists, it either prepares to change lane if it's safe or reduces the speed to avoid collision then follows the car ahead.


#### 4. Trajectory
This part of the code ***(line 379 to line 490)*** does the trajectory calculation based on the speed and lane output from the behavior, car coordinates and the past path points.

The last two points of the previous trajectory (or the car position if there are no previous trajectory) are used in conjunction three points (30m, 60m and 90m spaced points ahead of the starting reference) at a far distance to initialize the spline calculation.

For trajectory generation, we are using spline instead of polynomial trajectory generation. One of the reason is it is simple to use and requiring no dependencies. So we feed our future s values to the Spline library and it gives us a smoothed path of very close together waypoints. From there we just have to send these coordinates back to the simulation so that the car will follow them.

Below is a screenshot from the simulator for about 22 miles (30 minutes) without incidents:

![alt text][image1]

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
