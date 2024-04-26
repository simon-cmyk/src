#!/bin/bash

source /home/ttk4192/Documents/group4-temp-planner/bin/activate
rm -r plan
mkdir plan && cd plan
python2.7 /home/ttk4192/Documents/group4-temp-planner/temporal-planning/bin/plan.py stp-2 /home/ttk4192/catkin_ws/src/AI-planning/pddl-definitions/domain.pddl /home/ttk4192/catkin_ws/src/AI-planning/pddl-definitions/problem.pddl
cd ..
