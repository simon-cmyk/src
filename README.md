## Overview
The repository is split into 2 branches. The master branch is created for simulations in the Gazebo environment. The D0042 branch is for executing the AI-planner on the real robot. They differ mainly in map/obstacle definitions, and tuning of the controllers.




### How to launch Gazebo, TTK4192 assignment map and Turtlebot with manipulator
```bash
export TURTLEBOT3_MODEL=waffle_pi
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_ttk4192.launch
```

### How to launch the Turtlebot with a manipulator in the real environment
```bash
ssh user@ip
export TURTLEBOT3_MODEL=waffle_pi
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

###

### Known issues
* Hybrid A* is occasionally failing when computing a path. If so, then A* is implemented as a fallback path planner.
* The bringup launch file will spawn the robot at position (0,0) at angle 0 which is not in agreement with the internal map of the planner. Added A procedure to manually calibrate the robot before the AI-planner is ran. 

### Desired improvements
* Add functionality to re-plan based on changes in the environment
* Planner is using hard-coded euclidean distances in the AI-planner. It is desired to dynamically update these distances during run-time.
