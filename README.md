## Access token

Simon

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

Nicolas


Mathias

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

## How to launch Gazebo, TTK4192 assigment map and Turtlebot with manipulator
```bash
export TURTLEBOT3_MODEL=waffle_pi
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_ttk4192.launch
```
### Running log

ctrl + D in gazebo then you can record something.

Playback

```bash
gazebo -u -p /home/ntnu-itk/.gazebo/log/2024-04-09T162920.795934/gzserver/state.log
```

Spørsmål fredag 12.april

* [Tuesday 10:26 PM] Mathias Normann Knutsen
Simon; kan du høre på fredag hvordan vi skal løse denne oppgaven?
[Tuesday 10:26 PM] Mathias Normann Knutsen
du kan spørre om hvordan "the mission" som skal utføres blir definert med tanke på at vi har et kart fra slam og ikke et static map med kjent origo og kjente wpns
 like 1

* Må Man gå gjennom alle punktene?
* Invalid command name timer...

* GNC module handout
* When can we start with the physical one?

* ideas for reactive implementation? Replanning or? 
* Lab pc. Can we test on our own? 
* Reversing the car -> We only give the path -> Leads to funky behaviour.
Svar: 