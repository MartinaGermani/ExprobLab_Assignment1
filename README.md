# ExprobLab_Assignment1

### 1. Brief introduction to the project
In this repository, you can find a software architecture for planning and controlling the robot's behavior in a indoor environment for surveillance purposes.

The goal of the robot is to move between different locations in order to visit them and stay there for some times; in particular, the robot has to mainly stay in the corridors and move in a room if it becomes urgent because it has not been visited for some times. Moreover, when its battery is low, it has to move in the recharging station in order to recharge. 

### 2. Software Architecture
For implementing the robot's behavior described above, the State Machine is composed by three different states:
-   Move state, where the robot moves between corridors and rooms, when one of them becomes urgent,
-   Wait state, where the robot waits for some times in the current location,
-   Recharge state, in which the machine goes when the robot's battery becomes lows. In this state the robot moves in the recharging station (in this case located in room E) where it will recharge its battery.  

Regarding the building of the indoor environment's map, I created an ontology using Portégé editor, which then I loaded within the ROS architecture. In particular, a map of the indoor environment which I took into consideretion is below reported. 
