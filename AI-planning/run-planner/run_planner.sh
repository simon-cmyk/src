#!/bin/bash

source /home/ntnu-itk/catkin_ws/src/AI-planning/bin/activate
rm -r plan
mkdir plan && cd plan
python2.7 /home/ntnu-itk/temporal-planning/bin/plan.py stp-2 /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/domain.pddl /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/problem.pddl
cd ..
