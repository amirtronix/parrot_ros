

source /opt/ros/noetic/setup.bash
alias catkin="catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m"
export PYTHONPATH=$PYTHONPATH:/usr/src/ultralytics
source /catkin_ws/devel/setup.bash
