#!/usr/bin/env python
"""
.. module::StateAssignment
   :platform: Ubuntu 20:04
   :synopsis: this module represents the state machine of the architecture connected with the OWL ontology.
   
.. moduleauthor:: Martina Germani

The Finite-State Machine is composed by 3 different states:
	1) Move
	2) Wait
	3) Recharge
	
It manages the behaviour of the robot, which has to move between 
different locations in order to visit them and stay there for some times.
But, when the robot battery becomes low, it has to move in the recharging
station in order to recharge. 

In particular, the machine communicates with the ontology thanks to the
armor_py_api, which simplifies the calls to aRMOR. 

"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
import numbers
import math
import datetime
from armor_api.armor_client import ArmorClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from os.path import dirname, realpath
from std_msgs.msg import Bool
from assignment_exproblab import architecture_name_mapper as anm
from assignment_exproblab.fsm_helper_prova2 import InterfaceHelper
from assignment_exproblab.msg import Point, PlanAction, PlanGoal, ControlAction, ControlGoal
#from assignment_exproblab.srv import Location, LocationResponse


client = ArmorClient('armor_client', 'assignment')

LOG_TAG = anm.NODE_BEHAVIOUR


def load_ontology():
"""
Function for loading the ontology. 

It takes the path to the topological_map and send the path to the armor client.

"""
	path = "/root/ros_ws/src/topological_map/"
	client.utils.load_ref_from_file(path + "topological_map_abox.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False) 
	print("ontology loaded")

## function for moving in the corridors:
def move_corridor():
"""
Function for moving in the corridors.

It checks where the robot is. If the robot is already in a corridor, it has to remain there, otherwise
it checks if a corridor is reachable from the current position and it moves there. At this point
the timestamp of the current location will be updated with the current time of the machine. 

:returns: current position of the robot

"""
	current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
	if current_position[0][32:-1] == 'C1' or current_position[0][32:-1] == 'C2':
		print("the robot is already in a corridor")
		print("it remains here")
		time.sleep(2)
		new_value = time.time()
		updated_timestamp = update_time(current_position[0][32:-1], new_value)
		return current_position
	else:
		reachable_corridor = client.query.objectprop_b2_ind('canReach', 'Robot1')
		for i in range(len(reachable_corridor)):
			if reachable_corridor[i][32:-1]=='C1' or reachable_corridor[i][32:-1]=='C2':
				client.manipulation.replace_objectprop_b2_ind('isIn','Robot1', reachable_corridor[i][32:-1], current_position[0][32:-1])	
				client.utils.apply_buffered_changes()
				client.utils.sync_buffered_reasoner()
				new_current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
				print("robot now is in the corridor: ", new_current_position[0][32:-1])
				time.sleep(2)
				new_value = time.time()
				updated_timestamp = update_time(new_current_position[0][32:-1], new_value)
				return new_current_position
				

def set_initial_position():
"""
Function which sets the initial position of the robot.

By using the armor client, it set the initial position of the robot in the ROOM 'E'.

:returns: current position of the robot

"""
	client.manipulation.add_objectprop_to_ind('isIn','Robot1','E')
	client.manipulation.replace_objectprop_b2_ind('isIn','Robot1', 'E', '')
	client.utils.apply_buffered_changes()
	client.utils.sync_buffered_reasoner()
	position = client.query.objectprop_b2_ind('isIn', 'Robot1')
	print("robot starts in room: ", position)
	return position


def update_time(current_location, new_timestamp):
"""
Function which updates the visitedAt property of the location.

Anytime the robot visits a new location, then it's necessary to update the timestamp value of that location.
It requires the timestamp of the current location, it transforms it in a string of int and then, thanks to the armor client,
replace the old timestamp with the new one.

:returns: new timestamp of the current location

"""
	timestamp = client.query.dataprop_b2_ind('visitedAt', current_location)
	timestamp_int = int(timestamp[0][1:-11])
	timestamp_int = str(timestamp_int)
	new_timestamp = int(new_timestamp)
	new_timestamp =str(new_timestamp)
	client.manipulation.replace_dataprop_b2_ind('visitedAt', current_location, 'LONG', new_timestamp, timestamp_int)
	client.utils.apply_buffered_changes()
	client.utils.sync_buffered_reasoner()
	timestamp_2 = client.query.dataprop_b2_ind('visitedAt', current_location)
	return new_timestamp


def urgency_function(rooms):
"""
Function which determines which location is urgent to visit. 

:param rooms: list of available rooms
:returns: value of the urgency timestamp and of the corresponding urgent room

It check all the timestamps of the location and controls the value which is smaller. It return the smaller value of the timestamp 
and the associated room.
"""
	visited = 0
	room = ''
	i = 0
	for i in range(len(rooms)): 
		visited_room = client.query.dataprop_b2_ind('visitedAt', rooms[i][32:-1])
		if i == 0:
			visited = visited_room
			room = rooms[i]
		
		else:
			if visited_room[0][1:-11]<visited[0][1:-11]:
				print(visited_room[0][1:-11])
				print(visited[0][1:-11])
				visited = visited_room
				room = rooms[i]
			else:
				print(visited)
				print(room)

	return visited, room

def move_to_one(reachable_room):
"""
Function which moves the robot in a directly connected room.

:param reachable_room: location to reach
:returns: new current position of the robot

It checks the current position of the robot and, thanks to the armor client, replace the current position of the robot with 
the new position that it has to reach.
"""
	current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
	print("robot moves from ",current_position[0][32:-1], "to", reachable_room)
	boolean = client.manipulation.add_objectprop_to_ind('isIn','Robot1',reachable_room)
	client.manipulation.replace_objectprop_b2_ind('isIn','Robot1', reachable_room, current_position[0][32:-1])	
	client.utils.apply_buffered_changes()
	client.utils.sync_buffered_reasoner()
	new_current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
	print("robot now is in room: ", new_current_position)
	return new_current_position


# define state Move
class Move(smach.State):
"""
A class which defines the state Move, used to move the robot between different locations. 

The outcomes are:
1) 'battery_low': machine goes in Recharge state
2) 'wait_for': machine goes in Wait state
3) 'move_to': machine remains in this state

This is the initial state. 
First of all the robot checks if its battery is low or not by checking the topic state/battery_low. 
If the battery is low, it passes in the Charge state, otherwise it moves in the reachable corridor where it will check 
which location is urgent to be visited. Here there are different possibilities:
1) if the robot is already in the urgent room, it remains there and update the correspondent timestamp. Then it checks
the battery: if it is low, the robot passes in the Charge state, otherwise in the Wait state
2) if the urgent room is directly reachable because it is near the current location, the Planner is called, to which the urgent room is passed. So the Planner
will create a path to follow, and it will pass to the Controller, which simulates the robot motion by waiting for some times. In this way the robot moves from the
current location to the urgent one and again the corresponding timestamp is updated. Then it checks
the battery: if it is low, the robot passes in the Charge state, otherwise in the Wait state.
3) if the urgent room is not directly reachable but it is connected to the reachable one (i.e robot in room E has to reach R4), first of all it has to move
in the reachable location which is connected to the urgent one, in this way it can then move in the urgent one. Also in this case the planner/controller
mechanism is called. Then the robot checks the battery: if it is low, the robot passes in the Charge state, otherwise in the Wait state.


"""
    def __init__(self, interface_helper):
        # initialisation function, it should not wait
        self._helper = interface_helper
        
        smach.State.__init__(self, 
                             outcomes=['battery_low','wait_for','move_to'],
                             input_keys=['move_counter_in'],
                             output_keys=['move_counter_out'])
        
    def execute(self, userdata):
    	rooms = client.query.ind_b2_class('ROOM')
    	## first of all the robot checks if its battery is low or not:
    	## if battery is low, the robot changes its state and goes in recharge state
    	if self._helper.is_battery_low():
    		print("The battery is low. The robot can't move randomly and it needs to recharge")
    		return 'battery_low'
    	## otherwise it can remain in the move state and start to move on the basis of the urgency policy
    	else:
    		## robot check what location is urgent
    		urgency_timestamp, urgency_room = urgency_function(rooms)
    		## move in the near corridor
    		current_position = move_corridor()
    		current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
    		timestamp = client.query.dataprop_b2_ind('visitedAt', current_position[0][32:-1])
    		print("robot has to move in the urgency room: ", urgency_room, "whose timestamp is: ", urgency_timestamp)
    		## check what location is rechable
    		reachable_room = client.query.objectprop_b2_ind('canReach', 'Robot1')
    		print(reachable_room)
    		i = 0
    		j = 0
    		for i in range(len(reachable_room)):
    			print("room to reach: ", urgency_room[32:-1])
    		## if the robot can directly reach the urgent room:
    			if urgency_room[32:-1]==current_position[0][32:-1]:
    				print("the robot is already in the urgent room")
    				new_value = time.time()
    				updated_timestamp = update_time(urgency_room[32:-1], new_value)
    				if self._helper.is_battery_low():
    					print("The battery is low. The robot can't move randomly and it needs to recharge")
    					move_corridor()
    					return 'battery_low'
    				else:
    					return 'wait_for'

    				
    			elif urgency_room[32:-1] == reachable_room[i][32:-1]:
    				print("room directly reachable")
    				self._helper.send_planner_goal(urgency_room[32:-1])
    				self._helper.send_controller_goal()
    				rospy.sleep(1)
    				new_current_position = move_to_one(urgency_room[32:-1]) ## function which move the robot
    				new_value = time.time()
    				updated_timestamp = update_time(new_current_position[0][32:-1], new_value)
    				if self._helper.is_battery_low():
    					print("The battery is low. The robot can't move randomly and it needs to recharge")
    					move_corridor()
    					return 'battery_low'
    				else:
    					return 'wait_for'

    		## if the urgent room is in this moment not reachable:
    			elif urgency_room[32:-1] != reachable_room[i][32:-1]:
    			## check to which room the reachable location is connected
    				print("room not directly reachable")
    				print("current position: ", current_position)
    				print("reachable rooms: ", reachable_room)
    				connected_room = client.query.objectprop_b2_ind('connectedTo', reachable_room[i][32:-1])
    				print("connections available: ", connected_room)
    				
    				for j in range(len(connected_room)):
    			## if one of the connections corresponds to the urgency room
    					print(connected_room[j][32:-1])
    					print(urgency_room[32:-1])
    					if connected_room[j][32:-1] == urgency_room[32:-1]:
    						print("there is a connection, great")
    						self._helper.send_planner_goal(reachable_room[i][32:-1])
    						self._helper.send_controller_goal()
    						rospy.sleep(1)
    						new_current_position = move_to_one(reachable_room[i][32:-1])
    						print("now the robot is in room: ", new_current_position[0][32:-1])
    						print("robot moves from ",new_current_position[0][32:-1], "to the urgency room", connected_room[j][32:-1])
    						self._helper.send_planner_goal(connected_room[j][32:-1])
    						self._helper.send_controller_goal()
    						rospy.sleep(1)
    						new_current_position = move_to_one(connected_room[j][32:-1])
    						timestamp = client.query.dataprop_b2_ind('visitedAt', new_current_position[0][32:-1])
    						print("timestamp: ", timestamp)
    						new_value = time.time()
    						updated_timestamp = update_time(new_current_position[0][32:-1], new_value)

    						if self._helper.is_battery_low():
    							print("The battery is low. The robot can't move randomly and it needs to recharge")
    							move_corridor()
    							return 'battery_low'
    						else:
    							return 'wait_for'
 
    						
    						
# define state Wait
class Wait(smach.State):
"""
Class which defines the Wait state, in which the robot has to wait for some times in the current location.

The outcomes are:
1) 'battery_low': machine goes in Recharge state
2) 'wait_for': machine remains in Wait state
3) 'move_to': machine goes in Move state

When entered in this state, first of all the robot checks if its battery is low: 
1) if it is low the robot passes in the Recharge state
2) otherwise it remains in the Wait state where it will wait for some times in the current location, then it will
move in the reachable corridor and it will checks again the state of its battery: if it is low, it has to move
in the Recharge state, otherwise in the Move state where it will restart to move.

"""
    def __init__(self, interface_helper):
        # initialisation function, it should not wait
        self._helper = interface_helper
        smach.State.__init__(self, 
                             outcomes=['move_to','battery_low','wait_for'],
                             input_keys=['wait_counter_in'],
                             output_keys=['wait_counter_out'])
        
    def execute(self, userdata):
        # when entered in this state, again the robot first of all checks if its battery is low:
        ## if battery is low, the robot changes its state and goes in recharge state
    	if self._helper.is_battery_low():
    		print("The battery is low. The robot can't move randomly and it needs to recharge")
    		move_corridor()
    		return 'battery_low'
    	## otherwise it remains in the Wait state:
    	else:
        	current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
        	print("robot has to wait in the current location", current_position)
        	time.sleep(5)
        	move_corridor()
        	## after waiting, it checks again the state of its battery: in case it is low, it has to move in the Recharge state,
        	## otherwise it has to move in the Move state:
        	if self._helper.is_battery_low():
        		print("The battery is low. The robot can't move randomly and it needs to recharge")
        		return 'battery_low'
        	else:
        		print("robot's battery is not low. It is ready to restarts to move")
        		return 'move_to'
        
      
       

# define state Recharge
class Recharge(smach.State):
"""
Class which defines the Recharge state, where the robot moves for recharging its battery.

The outcomes are:
1) 'battery_low': machine remains in Recharge state
2) 'wait_for': machine goes in Wait state
3) 'move_to': machine goes in this state

The robot first of all checks the current position. Here there are 3 different possibilities:
1) if the robot is already in the recharging room, the recharging can start. In particular the recharge is simulated by waiting some seconds in the room. When the
recharge stops the timestamp of the recharging room is updated and the robot passes in the Move state. 
2) if the recharging room is directly reachable because it is near the current location, the Planner is called, to which the recharging room is passed. 
So the Planner will create a path to follow, and it will pass to the Controller, which simulates the robot motion by waiting for some times. 
In this way the robot moves from the current location to the recharging one where it will recharge its battery and again the corresponding timestamp is updated. 
Then it moves in the Move state.
3) if the recharging room is not directly reachable but it is connected to the reachable one (i.e robot in room R4 has to reach E), first of all it has to move
in the reachable location which is connected to the recharging one, in this way it can then move in the recharging one. Also in this case the planner/controller
mechanism is called. Again the corresponding timestamp is updated and then it moves in the Move state.


"""
    def __init__(self, interface_helper):
        # initialisation function, it should not wait
        self._helper = interface_helper
        smach.State.__init__(self, 
                             outcomes=['move_to','battery_low', 'wait_for'],
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        print("the battery of the robot is low, it has to recharge")
        recharging_room = 'E'
        current_position = client.query.objectprop_b2_ind('isIn', 'Robot1')
        if current_position[0][32:-1]==recharging_room:
        	print("charching starts")
        	time.sleep(10)
        	print("charging stops")
        	new_value = time.time()
        	updated_timestamp = update_time(current_position[0][32:-1], new_value)
        	return 'move_to'
        else:
        	reachable_room = client.query.objectprop_b2_ind('canReach', 'Robot1')
        	i = 0
        	for i in range(len(reachable_room)):
        		if recharging_room == reachable_room[i][32:-1]:
        			self._helper.send_planner_goal(recharging_room)
        			self._helper.send_controller_goal()
        			rospy.sleep(1)
        			new_current_position = move_to_one(recharging_room) ## function which move the robot
        			print("now the robot is in recharging room: ", reachable_room[i][32:-1])
        			print("charching starts")
        			time.sleep(10)
        			print("charging stops")
        			new_value = time.time()
        			updated_timestamp = update_time(new_current_position[0][32:-1], new_value)
        			return 'move_to'
        			
        		elif recharging_room != reachable_room[i][32:-1]:
        			connected_room = client.query.objectprop_b2_ind('connectedTo', reachable_room[i][32:-1])
        			j = 0
        			for j in range(len(connected_room)):
        				if connected_room[j][32:-1] == recharging_room:
        					self._helper.send_planner_goal(reachable_room[i][32:-1])
        					self._helper.send_controller_goal()
        					rospy.sleep(1)
        					new_current_position = move_to_one(reachable_room[i][32:-1])
        					print("now the robot is in room: ", new_current_position[0][32:-1])
        					print("robot moves from ",new_current_position[0][32:-1], "to the urgency room", connected_room[j][32:-1])
        					self._helper.send_planner_goal(connected_room[j][32:-1])
        					self._helper.send_controller_goal()
        					rospy.sleep(1)
        					new_current_position = move_to_one(connected_room[j][32:-1])
        					print("now the robot is in recharging room: ", new_current_position[0][32:-1])
        					print("charching starts")
        					time.sleep(10)
        					print("charging stops")
        					new_value = time.time()
        					updated_timestamp = update_time(new_current_position[0][32:-1], new_value)
        					return 'move_to'
    				

def main():
"""
Main function in which it initialised the ROS node and the SMACH state machine is created. 


"""
    rospy.init_node(anm.NODE_BEHAVIOUR, log_level = rospy.INFO)
    load_ontology()
    helper = InterfaceHelper()
    initial_position = set_initial_position()
   
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0
   
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(helper), 
                               transitions={'battery_low':'RECHARGE', 
                                            'wait_for':'WAIT',
                                            'move_to':'MOVE'},
                               remapping={'move_counter_in':'sm_counter', 
                                          'move_counter_out':'sm_counter'})
        smach.StateMachine.add('WAIT', Wait(helper), 
                               transitions={'move_to':'MOVE', 
                                            'battery_low':'RECHARGE',
                                            'wait_for': 'WAIT'},
                               remapping={'wait_counter_in':'sm_counter',
                                          'wait_counter_out':'sm_counter'})
        smach.StateMachine.add('RECHARGE', Recharge(helper), 
                               transitions={'move_to':'MOVE', 
                                            'battery_low':'RECHARGE',
                                            'wait_for': 'WAIT'},
                               remapping={'recharge_counter_in':'sm_counter',
                                          'recharge_counter_out':'sm_counter'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
			
