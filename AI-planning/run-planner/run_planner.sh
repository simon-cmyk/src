#!/bin/bash

source /home/ntnu-itk/AI-planning/bin/activate

python2.7 /home/ntnu-itk/AI-planning/temporal-planning/bin/plan.py stp-2 /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/domain.pddl /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/problem.pddl

