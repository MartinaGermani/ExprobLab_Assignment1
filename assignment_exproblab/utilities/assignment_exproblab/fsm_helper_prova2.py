#!/usr/bin/env python
"""
.. module:fsm_helper
   :platform: Ubuntu 20.04
   :synopsis: code used to manage different functionalities of the software architecture

.. moduleauthor:: Martina Germani

Initialization of parameters:
	client:ros_action_client
		armor client used to get and set information in the ontology
	
	planner_client:ros_action_client
		client of the motion/planner
	
	controller_client:ros_action_client
		client of the controller/planner

	subscriber:ros_msg_subscriber
		subscribs the battery state to the ros message
		
"""
# Import ROS libraries.
import rospy
from actionlib import SimpleActionServer, SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# Import constant names that define the architecture's structure.
from assignment_exproblab import architecture_name_mapper as anm

# Import ROS-based messages.
from std_msgs.msg import Bool
#from assignment_exproblab.msg import PlanAction
#from assignment_exproblab.srv import Location, LocationResponse
from assignment_exproblab.msg import PlanAction, ControlAction, PlanGoal, ControlGoal
from armor_api.armor_client import ArmorClient


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_BEHAVIOUR + '-HELPER'

class ActionClientHelper:
	def __init__(self, service_name, action_type, done_cb=None, feedback_cb = None, mutex=None):
		self.reset_client_states()
		
		self._service_name = service_name
		
		if mutex is None:
			self._mutex = Lock()
		else:
			self._mutex = mutex
		
		self._client = SimpleActionClient(service_name, action_type)
		
		self._external_done_cb = done_cb
		
		self._external_feedback_cb = feedback_cb
		
		self._client.wait_for_server()
		
	def send_goal(self, goal):
		if not self._is_running:
			self._client.send_goal(goal, done_cb = self._done_cb, feedback_cb = self._feedback_cb)
			self._is_running = True
			self._is_done = False
			self._results = None
		else:
			print("Warning!")
	
	def cancel_goals(self):
		if self._is_running:
			self._client.cancel_all_goals()
			self.reset_client_states()
		else:
			print("Warning2")
	
	def _feedback_cb(self, feedback):
		self._mutex.acquire()
		try:
			if self._external_feedback_cb is not None:
				self._external_feedback_cb(feedback)
		finally:
			self._mutex.release()
	
	def _done_cb(self, status, results):
		self._mutex.acquire()
		try:
			self._is_running = False
			self._is_done = True
			self._results = results
			if self._external_done_cb is not None:
				self._external_done_cb(status, results)
		finally:
			self._mutex.release()
	
	def is_done(self):
		return self._is_done
	
	def is_running(self):
		return self._is_running
	
	def get_results(self):
		if self._is_done:
			return self._results
		else:
			print("Error!")
	def reset_client_states(self):
		self._is_running = False
		self._is_done = False
		self._results = None
			
		
 
# A class to decouple the implementation of the Finite State Machine to the stimulus might that
# lead to state transitions. This class manages the synchronization with subscribers and action
# servers.
class InterfaceHelper:
    # Class constructor, i.e., class initializer.
	def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers.
        # Note that, based on different assumptions, further optimization can be done to make the different threads
        # blocking for a less amount of time in the same mutex.
        	self.mutex = Lock()
        # Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
        	self.reset_states()
        	
        	self.client = ArmorClient('armor_client', 'assignment')
        	        # Define the callback associated with the battery low ROS subscribers.
        	rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)
        	self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        	self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)
        	
        
	def reset_states(self):
		self._battery_low = False
        	
	def _battery_callback(self, msg):
       	 # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
		self.mutex.acquire()
		try:
            		# Get the battery level and set the relative state variable encoded in this class.
            		self._battery_low = msg.data
            # Uncomment below to log data.
            # if self._battery_low:
            #     log_msg = 'Robot with low battery.'
            # else:
            #     log_msg = 'Robot battery fully charged.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            		self.mutex.release()
	def is_battery_low(self):
      		return self._battery_low
      		
	def send_planner_goal(self, to_reach):
        	self.client.utils.apply_buffered_changes()
        	self.client.utils.sync_buffered_reasoner()
        	goal = PlanGoal(location_to_reach=to_reach)
        	print("goal inviato: ", goal)
        	self.planner_client.send_goal(goal)
        	print("ok")

        	
	def send_controller_goal(self):
		print("ciao")
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		path = self.planner_client.get_results()
		print("path passed to the controller: ", path)
		goal = ControlGoal(via_points = path.via_points)
		print("goal sent: ", goal)
		self.controller_client.send_goal(goal)
        	
  

	
     
