# Overview
The repository is split into 2 branches. The master branch is created for simulations in the Gazebo environment. The D0042 branch is for executing the AI-planner on the real robot. They differ mainly in map/obstacle definitions, and tuning of the controllers.

Some pictures can be found under [Image section](#images-from-the-project)  📸. And there are also videos from the Lab and Gazebo under [Video section](#videos-from-the-project)


## Additional packages
The STP-planner is an external module not included in the git repository. It must be fetched from the following repository before executing the AI-planne:
[temporal-planning](https://github.com/aig-upf/temporal-planning).

### How to launch the STP-planner 🚀
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
In three seperate terminals (after setting up the planner and modified the paths)
```bash
roscore
```

```bash
export TURTLEBOT3_MODEL=waffle_pi
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_ttk4192.launch
```

Then

```bash
cd catkin_ws/src/assignment4_ttk4192/script
rosrun assigment4_ttk4192 mission_planner_ttk4192.py 
```

If file not found, you need to run inside the `script` folder 
```bash
chmod +x *
```

### How to launch the Turtlebot with a manipulator in the real environment
```bash
ssh user@ip
export TURTLEBOT3_MODEL=waffle_pi
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

# Images from the project


| A* | Hybrid A* |
|-------|-------|
| <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/AstarPath.png" width="250"> | <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/pre2.png" width="250"> |
| Track | World |
| <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/post2.png" width="250" > | <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/Gazebo_World_ttk4192CA4.png" width="250"> |
| Maps | From Video |
| <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/maps.png" width="250" > | <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/example.png" width="250"> |
| Wpts | Taken by Turtlebot |
| <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/tweaked.png" width="250" > | <img src="https://raw.githubusercontent.com/simon-cmyk/src/master/images/waypoint2-25042024_094158.jpg" width="250"> |

# Videos from the project
These should be clickable links to the videos 📹
### Physical
[![Video](http://img.youtube.com/vi/De1WzHsptOs/0.jpg)](http://www.youtube.com/watch?v=De1WzHsptOs "AI Planning in Gazebo Simulation")

### Gazebo
[![Video](http://img.youtube.com/vi/5XR0a3griys/0.jpg)](http://www.youtube.com/watch?v=5XR0a3griys "Robot Manipulator Demo")

### Known issues
* Hybrid A* is occasionally failing when computing a path. If so, then A* is implemented as a fallback path planner is used. Hybrid A* can be tuned as specified in the top of the file `hybrid_a_star.py`.
* The bringup launch file will spawn the robot at position (0,0) at angle 0 which is not in agreement with the internal map of the planner. We added A procedure to manually calibrate the robot before the AI-planner is ran.
* path issues
### Desired improvements
* Functionality to re-plan based on changes in the environment
* Planner is using hard-coded euclidean distances. It is desired to dynamically update these distances during run-time.
* Add pose feedback in the GNC and path planning module to compensate for the robot's odometry drift.

### Credits :bow-tie:

| Contributor      | Email     |
| ---------------- | ---------------- |
| Mathias  |  [mathiank\@stud.ntnu.no ](mailto:mathiank@stud.ntnu.no )  |
| Nicolas  | [nicolarb\@stud.ntnu.no](mailto:nicolarb@stud.ntnu.no)   |
| Simon    | [simonlb\@stud.ntnu.no](mailto:simonlb@stud.ntnu.no)    |

