# ExprobLab_Assignment1

### 1. Brief introduction to the project
In this repository, you can find a software architecture for planning and controlling the robot's behavior in a indoor environment for surveillance purposes.

The goal of the robot is to move between different locations in order to visit them and stay there for some times; in particular, the robot has to mainly stay in the corridors and move in a room if it becomes urgent because it has not been visited for some times. Moreover, when its battery is low, it has to move in the recharging station in order to recharge. 

### 2. Software Behaviour
For implementing the robot's behavior described above, the State Machine is composed by three different states:
-   Move state, where the robot moves between corridors and rooms, when one of them becomes urgent,
-   Wait state, where the robot waits for some times in the current location,
-   Recharge state, in which the machine goes when the robot's battery becomes lows. In this state the robot moves in the recharging station (in this case located in room E) where it will recharge its battery.  

The state diagram is below reported, where the idea is that the robot starts moving from the starting room (in this case room E) to a reachable corridor, where it will wait until a room becomes urgent. In fact, when a room becomes urgent, the robot stops to wait and move in the room where it will wait for some seconds, after that it will return in the reachable corridor. The robot mantains this behavior unless its battery becomes low: in this case the robot goes to the recharging station, located in room E, where it will wait until its battery will be recharged, after that it will resume moving between the locations according to the logic described above.

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/state_machine.png?raw=true)

Regarding the building of the indoor environment's map, I created an ontology using Portégé editor, which then I loaded within the ROS architecture. In particular, a map of the indoor environment which I took into consideretion is below reported. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/map.jpg?raw=true)

So, as it's evident, the environment is composed by 3 different main classes:
- DOOR: D1, D2, D3, D4, D5, D6, D7
- LOCATION, which contains 2 subclasses: CORRIDOR (C1, C2) and ROOM (E, R1, R2, R3, R4)
- ROBOT: Robot1

To use this OWL ontology and its reasoner within ROS I used aRMOR [1], in particular armor_py_api [2], which simplifies the calls to aRMOR, but it can only be used from a python-based ROS node. 

### 3. Software Components
The software is composed of 4 nodes, each one available in the `scripts/` folder. 

### References
[1] https://github.com/EmaroLab/armor

[2] https://github.com/EmaroLab/armor_py_api
