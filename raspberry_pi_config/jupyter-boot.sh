#!/bin/bash

export PATH=/home/ubuntu/cling/bin:$PATH
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source /home/ubuntu/rosenv/bin/activate

jupyter notebook --no-browser
