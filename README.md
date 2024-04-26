## Overview
The repository is split into 2 branches. The master branch is created for simulations in the Gazebo environment. The D0042 branch is for executing the AI-planner on the real robot. They differ mainly in map/obstacle definitions, and tuning of the controllers.


### Additional packages
The STP-planner is an external module not included in the git repository. It must be fetched from the following repository before executing the AI-planne:
[temporal-planning](https://github.com/aig-upf/temporal-planning).

#### How to laucnh the STP-planner
- Follow the instructions in the [repository](https://github.com/aig-upf/temporal-planning) to install and build the planner outside the catkin workspace.
- Create a virtual environment for Python 2.7 outside the temporal-planning directory.
- To run the AI planner, you will need to customize the provided bash script (`AI-planning/run-planner/run_planner.sh`) with the appropriate absolute paths for your system. Follow these steps:

1. Open the `run_planner.sh` file in a text editor.

2. Locate the following lines in the script:

    ```bash
    source /home/ntnu-itk/catkin_ws/src/AI-planning/bin/activate
    rm -r plan
    mkdir plan && cd plan
    python2.7 /home/ntnu-itk/temporal-planning/bin/plan.py stp-2 /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/domain.pddl /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/problem.pddl
    cd ..
    ```

3. Replace the absolute path `/home/ntnu-itk/temporal-planning/bin/plan.py` with your path to the `plan.py` script in the temporal-planning repository.

4. Replace the absolute path `/home/ntnu-itk/AI-planning/temporal-planning/bin/activate` with the path to the virtual environment you created for Python 2.7.

5. Also change the root directory, `/home/ntnu-itk/` of the absolute path to `domain.pddl` and `problem.pddl` with the path to the catkin workspace.

6. Save the changes to the script.

7. Make the script executable, if it's not already:

    ```bash
    chmod +x run_planner.sh
    ```

8. Execute the script to run the AI planner:

    ```bash
    ./run_planner.sh
    ```


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
* Hybrid A* is occasionally failing when computing a path. If so, then A* is implemented as a fallback path planner is used.
* The bringup launch file will spawn the robot at position (0,0) at angle 0 which is not in agreement with the internal map of the planner. We added A procedure to manually calibrate the robot before the AI-planner is ran. 

### Desired improvements
* Functionality to re-plan based on changes in the environment
* Planner is using hard-coded euclidean distances. It is desired to dynamically update these distances during run-time.
* Add pose feedback in the GNC and path planning module to compensate for the robot's odometry drift.
