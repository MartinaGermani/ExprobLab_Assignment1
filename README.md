# ExprobLab_Assignment1

### 1. Brief introduction to the project
In this repository, you can find a software architecture for planning and controlling the robot's behavior in a indoor environment for surveillance purposes.

The goal of the robot is to move between different locations in order to visit them and stay there for some times; in particular, the robot has to mainly stay in the corridors and move in a room if it becomes urgent because it has not been visited for some times. Moreover, when its battery is low, it has to move in the recharging station in order to recharge. 

### 2. Software Components
The software is composed of 4 nodes, each one available in the `scripts/` folder. Below is reported the software architecture diagram.
![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/architecture_diagram.png?raw=true) 
### The `robot-state` node ###
The robot-state node implements a publisher of `Boolean` messages into the `state/battery_low` topic. This boolean value is published anytime the robot battery switches between the two possible states: low battery (i.e `True` is published) and recharged (i.e `False` is published). 
In particular, the simulation of the battery level is defined by a while-loop which modifies the boolean value in a random way, but you can also set the value of the delay variable in order to publish it using a specific delay. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/robot_state_diagram.png?raw=true) 
### The `Planner` node ###
The Planner node implements a `SimpleActionServer` named `motion/planner`. It requires as goal a string representing the location to reach, then it checks the current location of the robot and transforms the current and the target location from string to point coordinates thanks to the `anm.coordinates` parameter, that contains, for each location, the corresponding coordinates. Given the current and target points location, this component returns a plan as a list of `via_points`, which are randomly generated inside the range represented by the current and target points location. So, when a new `via_points` is generated, the updated plan is provided as `feedback`and when all the `via_points` have been generated, the plan is provided as `results`.

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/planner_diagram.png?raw=true) 
### The `Controller` node ###
The Controller node implements a `SimpleActionServer` named `motion/controller`. It requires as goal a list of `via_points` given by the `planner`. 
In particular, given the plan, this component waits fro each planned `via_points` and print the `via_points` on the screen in order to simulate the time spent for moving the robot to the location.
When the last `via_points` is reached, it provides a result.

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/controller_diagram.png?raw=true) 
### The `State-Assignment` node ###
The State-Assignment node implements the `Finite-State-Machine` which manages the behaviour of the robot. In particular it communicates with the ontology and with the reasoner of aRMOR through the use of `armor_py_api`: in this way the robot can move between the different locations defined in the ontology. Moreover this component calls the `planner` and the `controller` in order to simulate the motion of the robot in the environment, and it receives information about the robot battery from the robot-state node. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/state_assignment_diagram.png?raw=true)

### 3. Software Behaviour
For implementing the robot's behavior the State Machine is composed by three different states:
-   `Move` state, where the robot moves between corridors and rooms, when one of them becomes urgent,
-   `Wait` state, where the robot waits for some times in the current location,
-   `Recharge` state, in which the machine goes when the robot's battery becomes lows. In this state the robot moves in the recharging station (in this case located in room E) where it will recharge its battery.  

The state diagram is below reported, where the idea is that the robot starts moving from the starting room (in this case room E) to a reachable corridor, where it will wait until a room becomes urgent. In fact, when a room becomes urgent, the robot stops to wait and move in the room where it will wait for some seconds, after that it will return in the reachable corridor. The robot mantains this behavior unless its battery becomes low: in this case the robot goes to the recharging station, located in room E, where it will wait until its battery will be recharged, after that it will resume moving between the locations according to the logic described above.

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/state_machine.png?raw=true)

Regarding the building of the indoor environment's map, I created an ontology using Portégé editor, which then I loaded within the ROS architecture. In particular, a map of the indoor environment which I took into consideretion is below reported. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/map.jpg?raw=true)

So, as it's evident, the environment is composed by 3 different main classes:
- `DOOR`: D1, D2, D3, D4, D5, D6, D7
- `LOCATION`, which contains 2 subclasses: CORRIDOR (C1, C2) and ROOM (E, R1, R2, R3, R4)
- `ROBOT`: Robot1

To use this OWL ontology and its reasoner within ROS I used [aRMOR](https://github.com/EmaroLab/armor), in particular [ArmorPy API](https://github.com/EmaroLab/armor_py_api), which simplifies the calls to aRMOR, but it can only be used from a python-based ROS node.

### 4. Installation and running procedure
This architecture runs on ROS noetic. It also requires [aRMOR](https://github.com/EmaroLab/armor) and the [ArmorPy API](https://github.com/EmaroLab/armor_py_api).
Once you have all the installation required, you have to do the following steps:

- ```git clone https://github.com/MartinaGermani/ExprobLab_Assignment1.git ```
- go in the `scripts/` folder and run `chmod +x *`
- run `catkin_make` in the workspace
- once the workspace is built, open two terminals and execute the following commands:

```rosrun armor execute it.emarolab.armor.ARMORMainService```

```roslaunch assignment_exproblab assignment.launch```

### 5. Commented running
Here I reporte a video which shows how the architecture behaves:

https://user-images.githubusercontent.com/91315691/204513960-1cee8d3c-ab54-4c7d-813a-ff6d7996b1b3.mp4

*bottom-left*: **robot-state node**
*top-right*: **controller node**
*middle-right*: **planner node**
*bottom-right*: **state-assignment**

### 6. Working hypothesis and environment

### 7. Author and Contact
*Author*: **Germani Martina**

*Contact*: **martina.germani99@gmail.com**
