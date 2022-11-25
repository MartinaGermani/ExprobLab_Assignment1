# ExprobLab_Assignment1

### 1. Brief introduction to the project
In this repository, you can find a software architecture for planning and controlling the robot's behavior in a indoor environment for surveillance purposes.

The goal of the robot is to move between different locations in order to visit them and stay there for some times; in particular, the robot has to mainly stay in the corridors and move in a room if it becomes urgent because it has not been visited for some times. Moreover, when its battery is low, it has to move in the recharging station in order to recharge. 

### 2. Software Architecture
For implementing the robot's behavior described above, the State Machine is composed by three different states:
-   Move state, where the robot moves between corridors and one of the room, when one of them becomes urgent,
-   Wait state, where the robot waits for some times in the current location,
-   Recharge state, where the robot moves in the recharging station (in this case located in room E) when it
