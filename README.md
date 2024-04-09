## Access token

Simon

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

Nicolas


Mathias

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

## How to launch Gazebo, TTK4192 assigment map and Turtlebot with manipulator
```bash
export TURTLEBOT3_MODEL=waffle
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_ttk4192.launch
```
### Running log

ctrl + D in gazebo then you can record something.

Playback

```bash
gazebo -u -p /home/ntnu-itk/.gazebo/log/2024-04-09T162920.795934/gzserver/state.log
```

