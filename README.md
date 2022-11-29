# ExprobLab_Assignment1

### 1. Brief introduction to the project
In this repository, you can find a software architecture for planning and controlling the robot's behavior in a indoor environment for surveillance purposes.

The goal of the robot is to move between different locations in order to visit them and stay there for some times; in particular, the robot has to mainly stay in the corridors and move in a room if it becomes urgent because it has not been visited for some times. Moreover, when its battery is low, it has to move in the recharging station in order to recharge. 

### 2. Software Components
The software is composed of 4 nodes, each one available in the `scripts/` folder. Moreover there is an `architecture_name_mapper` interface which containes all the names of the parameters, topics and services used in the architecture. Below is reported the software architecture diagram.
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
In particular, given the plan, this component waits for each planned `via_points` and prints the `via_points` on the screen in order to simulate the time spent for moving the robot to the location.
When the last `via_points` is reached, it provides a result.

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/controller_diagram.png?raw=true) 
### The `State-Assignment` node ###
The State-Assignment node implements the `Finite-State-Machine` which manages the behaviour of the robot. In particular it communicates with the ontology and with the reasoner of aRMOR through the use of `armor_py_api`: in this way the robot can move between the different locations defined in the ontology. Moreover this component calls the `planner` and the `controller` in order to simulate the motion of the robot in the environment, and it receives information about the robot battery from the robot-state node. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/state_assignment_diagram.png?raw=true)

### 3. Software Behaviour
For implementing the robot's behavior the State Machine is composed by three different states:
-   `Move` state, where the robot moves between corridors and rooms, when one of them becomes urgent
-   `Wait` state, where the robot waits for some times in the current location
-   `Recharge` state, in which the machine goes when the robot's battery becomes lows. In this state the robot moves in the recharging station (in this case located in room E) where it will recharge its battery.  

The state diagram is below reported, where the idea is that the robot starts moving (`Move` state) from the starting room (in this case room `E`) to a reachable corridor, where it will checks if some room is urgent. 
When a room becomes urgent, the robot moves in the urgent room where it will wait for some seconds (`Wait` state), after that it will return in the reachable corridor. 
Anytime the robot visits a location it will update the `timestamp` value associated to that location (this value represents the last time the robot visited that location). 
The robot mantains this behavior unless its battery becomes low: in this case, if it's not already there, the robot goes to the recharging station, located in room `E`, where it will wait until its battery will be recharged, after that it will resume moving between the locations according to the logic described above.
Moreover, thanks to some controls implemented, the robot is able to reach any location in the environment by checking where is it and what are the possible connections between the locations. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/state_machine.png?raw=true)

Regarding the building of the indoor environment's map, I created an ontology using Portégé editor, which then I loaded within the ROS architecture. In particular, a map of the indoor environment which I took into consideretion is below reported. 

![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/map.jpg?raw=true)

So, as it's evident, the environment is composed by 3 different main classes:
- `DOOR`: D1, D2, D3, D4, D5, D6, D7
- `LOCATION`, which contains 2 subclasses: CORRIDOR (C1, C2) and ROOM (E, R1, R2, R3, R4)
- `ROBOT`: Robot1

To use this OWL ontology and its reasoner within ROS I used [aRMOR](https://github.com/EmaroLab/armor), in particular [ArmorPy API](https://github.com/EmaroLab/armor_py_api), which simplifies the calls to aRMOR, but it can only be used from a python-based ROS node.

For simulating the motion of the robot I created a parameter called `coordinates` which contains the point coordinates which I set for each location in the environment. These coordinates will be used by the planner for planning the path which the robot should follow in order to reach the desired location. The following map shows the points associated to each location.
![alt text](https://github.com/MartinaGermani/ExprobLab_Assignment1/blob/main/diagrams/map_points.jpg?raw=true)

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
Here I reported a video which shows how the architecture behaves:

https://user-images.githubusercontent.com/91315691/204513960-1cee8d3c-ab54-4c7d-813a-ff6d7996b1b3.mp4

*bottom-left*: **robot-state node**

*top-right*: **controller node**

*middle-right*: **planner node**

*bottom-right*: **state-assignment**

After loaded the ontology, the robot starts in room E. The initial machine state is the `Move` state, so the robot moves in one of the reachable corridor where it will checks if some rooms is urgent to be visited. If there is an urgent room and the robot battery is charged, the state-assignment node sends the location to reach to the planner, which creates the via_points and sends it the result, then this result will be passed to the controller which simulates the robot motion, and then the ontology will be updated replacing the new current position of the robot. At this point it will check if the battery is low, so if it is low the machine switches to the `Recharge` state, otherwise in the `Wait` state.
If the machine enters in the `Recharge` state, first of all it checks the currrent position of the robot: if the robot is already in the recharging station (i.e room E), it can start the recharging (simulated by passing some seconds), otherwise first it has to move in the recharging room. Once the recharging is finished, the machine passes in the `Move` state assuming a behaviour as above described.
If the machine enters in the `Wait` state, it rechecks if the robot battery is low and if it is charged it simulate a waiting by passing some seconds in the current location. 

### 6. Working hypothesis and environment
Working hypothesis and environment:
- the location `E` is a `ROOM`
- the robot starts in room `E` with fully recharged battery
- the position of each `ROOM` is setted as described in the `coordinates` parameter inside the `architecture_name_mapper` interface.
### 7. Limitations
System limitations:
- there is not a physical model for the motion but it is only simulated
- the loading of the ontology is static and it is based on a known environment, while a possible improvement could consist in creating the ontology in real-time as the robot moves in the environment
- the sequence of urgent rooms to visit is fixed having set in advance a timestamp for each location, a value which will then be updated as the robot visits the location itself. 
- I set only a point for each location, so in the simulation the robot doesn't move really inside the location but it only arrives in a fixed point associated to the location. 

### 8. Author and Contact
*Author*: **Germani Martina**

*Contact*: **martina.germani99@gmail.com**
