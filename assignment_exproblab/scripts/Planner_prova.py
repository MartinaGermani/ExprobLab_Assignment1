#!/usr/bin/env python

"""
.. module::Planner
   :platform: Ubuntu 20:04
   :synopsis: this module represents an action server to simulate motion planning.
   
.. moduleauthor:: Martina Germani



Action Server for simulating motion planning.

Given a location to reach, it checks the current position of the robot thanks
to the armor client and it finds the corresponding coordinate points of the 
locations set in the anm.coordinates parameter. 
At this point the planner create in a random way a set of via_point building
a plan for passing from the current position to the target one. This plan is returned. 

"""


import random
import rospy
import actionlib
from random import randint
from actionlib import SimpleActionServer
from armor_api.armor_client import ArmorClient
from assignment_exproblab import architecture_name_mapper as anm
from assignment_exproblab.msg import Point, PlanFeedback, PlanResult, PlanGoal
import assignment_exproblab 

# a tag for identifying logs producer
LOG_TAG = anm.NODE_PLANNER

class ActionPlanner(object):

	def __init__(self):
		self.client=ArmorClient('armor_client', 'assignment')
		
		self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
		
		self._actionserver = SimpleActionServer(anm.ACTION_PLANNER, assignment_exproblab.msg.PlanAction, execute_cb = self.execute_callback, auto_start = False)
		self._actionserver.start()

	def execute_callback(self, goal):
		current_location = self.client.query.objectprop_b2_ind('isIn', 'Robot1')
		current_location = current_location[0][32:-1]
		current_location_point = anm.coordinates[current_location]
		
		reach_location_point = anm.coordinates[goal.location_to_reach]
		print("coordinates of the location to reach: ", reach_location_point)
		feedback = PlanFeedback()
		feedback.via_points=[]
		
		new_point = Point()
		new_point.x = random.uniform(current_location_point[0], reach_location_point[0])
		new_point.y = random.uniform(current_location_point[1], reach_location_point[1])
		
		feedback.via_points.append(new_point)
		self._actionserver.publish_feedback(feedback)
		
		result = PlanResult()
		result.via_points = feedback.via_points
		self._actionserver.set_succeeded(result)
		log_msg = 'Motion plan succeeded with plan: '
		log_msg +=''.join('('+str(point.x)+','+str(point.y)+')' for point in result.via_points)
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

if __name__=='__main__':
	rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
	server = ActionPlanner()
	rospy.spin()
