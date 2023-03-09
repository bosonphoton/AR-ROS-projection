#!/usr/bin/env python

from parser import Policy,Solver
from pomdp_parser import Model
# from pomdp_solver import generate_policy
import numpy as np
np.set_printoptions(precision=2)     # for better belief printing
import random
import pathlib
import pandas as pd
import os
########################
from get_dialogue import hear_color, ask_to_look, hear_confirmation
import rospy
import actionlib

import time
#from ur5_init_node_av.py import MoveGroupInteface
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

import subprocess
import sys

import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
import std_srvs.srv
from std_msgs.msg import String, Float32, Empty
import time

current_table_number = None

class Simulator:


	def __init__(self, solve_pomdp=False):

		#path to pomdp file stored
		pomdp_file =  '/home/turtlebot/appl/src/model_update_2.pomdp'
		assert pathlib.Path(pomdp_file).is_file(), 'POMDP path does not exist'

		solver_path = '/home/turtlebot/appl/src/pomdpsol'
		assert pathlib.Path(solver_path).is_file(), 'Solver path does not exist'

		self.model = Model(pomdp_file=pomdp_file, parsing_print_flag=True)
		print(self.model.states)
		print(self.model.actions)
		print(self.model.trans_mat.shape)

		policy_file = '/home/turtlebot/appl/src/out.policy'
		print (policy_file)
		# if solve_pomdp:
		# 	generate_policy(solver_path,pomdp_file,policy_file)
		# #assert pathlib.Path(policy_file).is_file(), 'POLICY path does not exist'

		self.policy = Policy(len(self.model.states),
							len(self.model.actions),
							policy_file=policy_file)


	def update(self, a_idx,o_idx,b ):
		'''Update belief using Bayes update rule'''
		b = np.dot(b, self.model.trans_mat[a_idx, :])
		b = [b[i] * self.model.obs_mat[a_idx, i, o_idx] for i in range(len(self.model.states))]
		b = b / sum(b)

		return b

	def pretty_print(self,b):
		df = pd.DataFrame(b,index=False, columns=self.model.states)
		print (df)


	def observe(self,a_idx,next_state):


		if a_idx == 0: #gaze
			ask_to_look()
			while(current_table_number == None):
				print "Waiting for gaze input"
				time.sleep(1)
			return current_table_number


		elif a_idx == 1: #ask color
			dialogue_c = hear_color()
			return dialogue_c

		elif a_idx == 2: #project AR & do robot stuff
			projectARObject.publish("red_1")
			# pass
			#project red 1

		elif a_idx == 3: #project AR
			projectARObject.publish("blue_1")
			# pass
			#project blue 1

		elif a_idx == 4: #project AR
			projectARObject.publish("red_2")
			# pass
			#project red 2

		elif a_idx == 5: #project AR
			projectARObject.publish("blue_2")
			# pass
			#project blue 2



		# elif a_idx == 7:
		# 	#arm.plan_joint_goal(0,-94,74,-78,-89,-8.9)
		# 	script1 = "/home/chelsea/catkin_ws/src/ur5_yan/scripts/pomdp/ur5_init_node1.py"
		# 	subprocess.call(['python', script1], stdout=sys.stdout, stderr=subprocess.STDOUT)
		# 	return "na"

		# elif a_idx == 8:
		# 	#arm.plan_joint_goal(0,-94,74,-78,-89,-8.9)
		# 	script2 = "/home/chelsea/catkin_ws/src/ur5_yan/scripts/pomdp/ur5_init_node2.py"
		# 	subprocess.call(['python', script2], stdout=sys.stdout, stderr=subprocess.STDOUT)
		# 	return "na"

		# elif a_idx == 9:
		# 	#arm.plan_joint_goal(0,-94,74,-78,-89,-8.9)
		# 	script3 = "/home/chelsea/catkin_ws/src/ur5_yan/scripts/pomdp/ur5_init_node3.py"
		# 	subprocess.call(['python', script3], stdout=sys.stdout, stderr=subprocess.STDOUT)
		# 	return "na"

		# elif a_idx == 10:
		# 	#arm.plan_joint_goal(0,-94,74,-78,-89,-8.9)
		# 	script4 = "/home/chelsea/catkin_ws/src/ur5_yan/scripts/pomdp/ur5_init_node4.py"
		# 	subprocess.call(['python', script4], stdout=sys.stdout, stderr=subprocess.STDOUT)
		# 	return "na"


	def run(self):

		global current_table_number
		#Initialize belief (randomly select states)
		b = np.ones(len(self.model.states))/(len(self.model.states)-1)
		b[-1]=0.0

		print (b)

		term=False
		reset = False
		state= np.random.choice(self.model.states[:-1])  # do not sample term

		while not term:
			a_idx=self.policy.select_action(b)

			s_idx = self.model.states.index(state)
			print ('\n\n\nUnderlying state: ', state)
			print ('action is: ',self.model.actions[a_idx])
			current_action = self.model.actions[a_idx]


			next_state = np.random.choice(self.model.states, p=self.model.trans_mat[a_idx,s_idx,:])
			obs = self.observe(a_idx,next_state)
			print("Next state", next_state)
			print obs

			if next_state == "terminal":
				dialogue_confirmation = hear_confirmation()
				if(dialogue_confirmation == "yes"):
					term = True
					if((current_action == "projectred1") or (current_action == "projectblue1")):
						print "Going to location 1"
						#goto_location("1.73,0.888,0.0034,0.0",0) # Table 1
						#goto_location("-1.62,0.425,0.0034,0.0",0) # Target table
						term = False
						reset = True
						current_table_number = None

						instance= Simulator()
						instance.run()
					else:
						print "Going to location 2"
						#goto_location("1.85,-1.58,0.0034,0.0",0) # Table 2
						#goto_location("-1.62,0.425,0.0034,0.0",0) # Target table
						term = False
						reset = True
						current_table_number = None
						instance= Simulator()
						instance.run()
					continue
				elif (dialogue_confirmation == "no"):
					print "Removing last"
					projectARObject.publish("remove_last")
					next_state = ""
					term = False
					reset = True
					current_table_number = None
					instance= Simulator()
					instance.run()

			if not reset:
				obs_idx = self.model.observations.index(obs)
				print ('observation is: ',self.model.observations[obs_idx])
				b = self.update(a_idx,obs_idx,b)
				print(b)

				state = next_state
				if b[-1]>0:
					term=True
					print('\n')


def goto_location(location,sensitivity):
    goalReached = False

    while(not goalReached):

        goalReached = moveToGoal(location,sensitivity)
        if (goalReached):
            rospy.loginfo("Reached destination!")


def moveToGoal(location,sensitivity):
    global move
    move = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    while(not move.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
    simplegoal = MoveBaseGoal()
    simplegoal.target_pose.header.frame_id = "map"
    simplegoal.target_pose.header.stamp = rospy.Time.now()
    goal_location_coordinates = location.split(",")
    simplegoal.target_pose.pose.position =  Point(float(goal_location_coordinates[0]),float(goal_location_coordinates[1]),0)
    simplegoal.target_pose.pose.orientation.x = 0.0
    simplegoal.target_pose.pose.orientation.y = 0.0
    # if(sensitivity != 0):
    simplegoal.target_pose.pose.orientation.z = float(goal_location_coordinates[2])
    simplegoal.target_pose.pose.orientation.w = float(goal_location_coordinates[3])
    # else:
    #     simplegoal.target_pose.pose.orientation.z = 0.0
    #     simplegoal.target_pose.pose.orientation.w = 0.0
    rospy.loginfo("Sending Next goal location ")
    move.send_goal(simplegoal)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    msg = Twist()
    speed = 0.1
    msg.linear.x = speed
    pub.publish(msg)

    move.wait_for_result(rospy.Duration(60))
    if(move.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("This is the end of navigation")
        return True

    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


def getTableGaze(data):
	global current_table_number
	current_table_number = str(data.data)


def main():
	instance= Simulator()
	instance.run()
	print "Complete"



if __name__ == '__main__':

	try:
		rospy.init_node('simulatortest')
		rate = rospy.Rate(10) # 1hz
		projectARObject = rospy.Publisher('object_to_project', String, queue_size=10)
		rospy.Subscriber("/gaze_table", String, getTableGaze)
		main()
		# rospy.spin()

	except rospy.ROSInterruptException:
		print("finished!")
