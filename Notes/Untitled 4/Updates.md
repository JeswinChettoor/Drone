# September 7
- Realised laser odometry and all of that was not needed
- Removing all of that 
- Changing params to rely only on ekf3 from the drone

## September 8
#### Things to do 
- Goal is to get slam toolbox running + get the drone to fly in guided or guided no gps mode.
- Also make sure rangefinder is available to ardupilot 
- Handle conversions.
- Install mission planner
#### Done
- Removed everything back to only have ros gazebo bridge. 
- Removed some params which were not needed.
- Talked to anant  about  guided mode .
- Fixed params to use barometer for height instead of rangefinder , in irl , it would be just one param change. Ardupilot also handles tilt compensation for rangefinder.
- Read slam toolbox documentation
- Set up mavros again
- Looked into how laser odometry is not needed , but just tune slam pose properly.
### September 9
## Things to do
- Set up transforms.
- Get slam toolbox running
- Changed design of the localization .
- Now rf20  is only giving odom->base_link.
- No fusing ekf on companion
- Figured out better ways for tilt  compensation using pointclouds  and ros
### Things done
- Scan_stabilized running
- got transforms runnning 
- Launch files
## September 10
- Verified scan_stabilized s accuracy
- Got slam toolbox running
- Was facing message dropping issues due to both sim time not being followed everywhere. also  update rate is very low , around 3hz. Had to lower scan to match the rf20 speed.
- 