# WELCOME TO RAMMSTEIN ASTROBEE MANEUVER
This package should be built alongside Astrobee. The goal of the executable are fairly straightforward:
- Leader Maneuver can perform a square or line maneuver
- Follower Maneuver will chase a target while respecting the min distance boundaries 
## Building
### Dependencies 
Astrobee should have natively all the required dependencies to build the code, but feel free to check the CMakeLists for further detail. Note that you'll need to have astrobee_ws/devel/setup.bash sourced before building!

### Compile and Run
- Extract the zip to your ROS_WS, i.e., folder_ws->src->rammstein
- catkin_make

Terminal 1
````bash
cd $folder_ws
source devel/setup.bash
roslaunch rammstein follwer.launch ## for the follower
roslaunch rammstein leader.launch ## for the leader
````
Do not run both launches in simulation, if you want to do so, make sure to edit the topics to reflect each bot namespace.
### Troubleshooting
- Open .launch files to change parameters as you wish
