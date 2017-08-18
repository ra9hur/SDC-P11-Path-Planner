# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

---

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

---

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

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

---

## Model Documentation

The goal of the project is to define a path made up of {X,Y} points that the car will visit sequentially every 0.02 seconds. The model implemented is based on Finite State Machine which is used to determine the behavior of the autonomous vehicle. Every time a certain behavior is chosen, a Trajectory is constructed which executes that behavior.

The model receives a data update from simulator 50 times a second. Every time this update is received, the model also updates the path plan with new position(s) and sends it back to the simulator. This plan reaches '50 time-steps' into the future, corresponding to 1 second (50 time-steps * 1/50 second update rate). Each time-step is about 0.5m and the total distance for 50 time-steps is 25m. It is assumed that other vehicles on the road do not change their speed or lane during this 1s. Usually the plan is not created from scratch, but consists of the plan from previous update minus the position of 1 time-step.

A typical update cycle looks like this: 

* The model first receives the data from the simulator.     `Source: main.cpp; main(); h.onMessage()`
	
  This data consists of positions and speed of the autonomous vehicle, the positions and speed 	of the other vehicles and the remainder of the previous path plan (i.e. the part which has not yet been executed).

* Prediction                                                `Source: PathPlanner.cpp`
  
  * PathPlanner::MaintainListSurroundingCars
	  * This functionality uses sensor data of other cars on the road and creates a new list that maintains cars that are 50m from the ego car.
	  * Velocity of each car and their s distance from the ego car are additionally computed and saved into the list for reference in the subsequent steps.
	  * Cars are segregated based on the lanes that they are currently moving on and also based on their position (ahead / behind) wrt ego car.

  * PathPlanner::CheckChangeSpeed
    * Checks if there are cars ahead of the ego car. 
    * Based on the response, it is decided whether to slow down, switch lanes or just drive ahead with no changes.

  * PathPlanner::AverageLaneSpeed
    * This function is invoked when CheckChangeSpeed function detects that there is a car ahead of ego car.
    * If the ego car needs to be slowed down, the speed is reduced based on the speed of the car ahead.

  * PathPlanner::CheckOtherCarLC
    * As the ego car drives through, there are chances that other cars at the front in other lanes switches into the lane that the ego car is driving on.
    * Also there are scenarios where in, ego car detects a car ahead and decides to switch lanes. Just then, the other car also decides to switch lanes resulting in a collision.
    * This function predicts if the car ahead is switching lanes. This helps to slow down the ego car (or not to change lanes) and avoid collision.

* Behavioural Planning                                      `Source: PathPlanner.cpp`
  
  * PathPlanner::PrepareLCL
    * This function is invoked when there is a car ahead and the ego checks to switch lanes to the left to overtake.
    * Checks, if it is safe to change lanes (using IsLaneChangeSafe function).
    * Also checks, if the car ahead is changing lanes (using CheckOtherCarLC function).
    * If it is safe to change lanes and the car is not changing lanes, then then let the trajectory planner know that it is safe to switch lanes to overtake.

  * PathPlanner::PrepareLCR
    * This function is invoked when there is a car ahead and the ego car checks to switch lanes to the right to overtake.
    * Functionality is similar to that described for PrepareLCL function.

  * PathPlanner::IsLaneChangeSafe
    * This functions checks if it is safe for the ego car to move into the new lane
    * The following scenarios are checked on the new lane
      * Are there any cars at the front OR at the back of the ego car ?
      * There is no car at the front, there is a car at the back. Is the s distance greater than 25m from the ego car ?
      * There is a car at the front, but no car at the back. Is the s distance greater than 15m from the ego car ?
      * There are cars at the front and back of the ego car. Are these two cars 65m (25m at the back + 25m for 1 second projection + 15m at the front) apart for the ego car to move into the gap ?

* Generate Trajectory                                      `Source: main.cpp`
  * Initially when the path plan is empty, a new path plan is created using car's initial state. This is when the car is just starting and does not have previous points for reference. Previous point is calculated using car's yaw angle. These two points make a path that is tangent to the car. To these points, 30m spaced points ahead of the starting refereence (in frenet coordinates) are added. These points represent waypoints in the map coordinate system. They are transformed to local coordinates and then a spline is created using 5 points. For the initial iteration, all 50 points are chosen using spline. All the chosen points are transformed back to global coordinates and then appended to the path plan.
  * In every iteration, simulator consumes only a few path points and remaining ones are available for the next iterations. Remaining points are available as previous path points. Additionally, new points are generated and added to the path plan. Last two end-points of the previous path are used as reference to find the yaw_angle. To these points, 30m spaced points ahead of the starting refereence (in frenet coordinates) are added. These points are transformed to local coordinated system to create a spline. New points are generated using the created spline, transformed to global coordinates and are appended to the previous path points to generate a new path plan.
  * For every iteration of the loop, it is first determined how many 'time-steps' the path plan is ahead of the actual position of the autonomous vehicle. The Path Planner is then tasked with finding the closest vehicles given the 'time-steps' ahead. So, for example, if the path plan is 50 'time-steps' ahead,then the Path Planner is tasked to find the closest vehicles 1 second in the future. Though, this method is just a guestimate at best.
  * The reference velocity is initially set to zero and then incremented in the iteration loop till it hits 49.5 mph. Each increment is about 1 mps (or 0.224 mph) to ensure gradual increase and thereby, avoiding jerks. If the ego car sees a car ahead, the velocity is gradually decremented by 1 mps till the velocity reaches that of the car ahead to maintain lane velocity.
  * The iteration loop receives information from behaviour planner on changing speed or lanes. These values are used as inputs to the path plan. The loop is now ended. The new path plan is then sent to the simulator and the update cycle is ended.

---

## Demo Video

A video showing the project demo can be found at [Youtube](https://youtu.be/NtTLMdKljq4)

---

