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
		'''Make an observation using random distribution of the observation marix'''
		s_idx = self.model.states.index(next_state)
		
		return np.random.choice(self.model.observations, p= self.model.obs_mat[a_idx,s_idx,:]) 

	def run(self):

		#Initialize belief (randomly select states)
		b = np.ones(len(self.model.states))/(len(self.model.states)-1)
		b[-1]=0.0

		print (b)
		
		term=False
		state= np.random.choice(self.model.states[:-1])  # do not sample term
		
		while not term:
			a_idx=self.policy.select_action(b)
			
			s_idx = self.model.states.index(state) 
			print ('\n\n\nUnderlying state: ', state)
			print ('action is: ',self.model.actions[a_idx])
			

			
			next_state = np.random.choice(self.model.states, p=self.model.trans_mat[a_idx,s_idx,:])
			obs = self.observe(a_idx,next_state) #next_state or state?
			print("Next state", next_state)
			obs_idx = self.model.observations.index(obs)
			print ('observation is: ',self.model.observations[obs_idx])
			b = self.update(a_idx,obs_idx,b)
			print(b)
			
			state = next_state
			if b[-1]>0:
				term=True
				print('\n')

def main():
	instance= Simulator()
	instance.run()




if __name__ == '__main__':
	main()
