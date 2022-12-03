#!/usr/bin/env python


"""
.. module::Controller
   :platform: Ubuntu 20:04
   :synopsis: this module represents an action server to simulate motion controlling.
   
.. moduleauthor:: Martina Germani


Action Server for simulating motion controlling. 
It takes the plan as a set of via_points and it simulates the motion by
waiting some times for each point of the path.

"""
import rospy
import random
from actionlib import SimpleActionServer
from armor_api.armor_client import ArmorClient
from assignment_exproblab import architecture_name_mapper as anm
from assignment_exproblab.msg import Point, ControlAction, ControlFeedback, ControlResult, ControlGoal
import assignment_exproblab

LOG_TAG = anm.NODE_CONTROLLER

class ActionController(object):

	def __init__(self):
		self.client=ArmorClient('armor_client', 'assignment')
		# instantiate and start the motion action server
		self._actionserver = SimpleActionServer(anm.ACTION_CONTROLLER, assignment_exproblab.msg.ControlAction, execute_cb = self.execute_motion_callback, auto_start=False)
		self._actionserver.start()
		
		
	def execute_motion_callback(self, plan):
		print("controller")
		feedback = ControlFeedback()
		for point in plan.via_points:
			# check that the client didn't cancel this service
			if self._actionserver.is_preempt_requested():
				print("Service has been cancelled by the client")
				self._actionserver.set_preempted()
				return
			
			print(point)
			print("wait in order to simulate the transition")
			rospy.sleep(2)
			feedback.reached_point = point
			self._actionserver.publish_feedback(feedback)
	
		
		result = ControlResult()
		result.reached_point = feedback.reached_point
		self._actionserver.set_succeeded(result)
		return

		
if __name__=='__main__':
	rospy.init_node(anm.NODE_CONTROLLER, log_level= rospy.INFO)
	server = ActionController()
	rospy.spin()
				
