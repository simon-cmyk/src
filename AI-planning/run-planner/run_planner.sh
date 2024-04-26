#!/bin/bash

# Here you have to create a virtual environment for python2.7
# and insert the path to the activate file.
source /home/ntnu-itk/catkin_ws/src/AI-planning/venv/bin/activate

# insert the plan.py path and the path to the pddl-definitions
rm -r plan

mkdir plan && cd plan
python2.7 /home/ntnu-itk/temporal-planning/bin/plan.py stp-2 /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/domain.pddl /home/ntnu-itk/catkin_ws/src/AI-planning/pddl-definitions/problem.pddl
cd ..
