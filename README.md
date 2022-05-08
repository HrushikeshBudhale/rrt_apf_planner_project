# RRT+APF Path Planner

A closed loop path plannign and following for Turtlebot3 robot in Gazebo simulator using RRT+APF path planner and differential drive controller.

- RRT - Rapidly Exploring Random Trees
- APF - Artificial Potential Field



<p align="center">
  <img src="https://github.com/HrushikeshBudhale/rrt_apf_planner_project/blob/master/docs/rrt_apf_planner.gif?raw=0" alt="OUTPUT GIF" width="300"/>
</p>

 Demo video with gazebo simulation [youtube](https://youtu.be/93RBWRq_yuU)
 
## Dependencies
- python 3.6 or above
- numpy
- matplotlib
- opencv
- turtlebot3


## Install Turtlebot3
Install turtlebot3 packages using instructions
from [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

## How to run?

Clone the project folder in your catkin workspace and build the package using following commands

```
cd catkin_ws/src/
git clone https://github.com/HrushikeshBudhale/rrt_apf_planner_project.git
cd ..
export TURTLEBOT3_MODEL=burger
catkin build rrt_follower
source devel/setup.bash
```
Launch the nodes using following command
```
roslaunch rrt_follower turtlebot3_map.launch
# or
roslaunch rrt_follower turtlebot3_map.launch start_x:=1 start_y:=1 start_angle:=0 goal_x:=9 goal_y:=9
```

On launching the gazebo simulator, running this node will first plan the path and then user can see the the robot moving towards goal location.

## Misc.

- Efficient use of vectorized arrays takes less than 0.5 sec to find path between 2 farthest points in the map.