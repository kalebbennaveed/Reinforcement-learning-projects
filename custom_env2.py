# This env is made for the purposes of testing in the Argo AI Centre of Autonomous researc
# Simple scenrio which will be implemented in this scenrio
# one car acts a blockade on the left side of the car and the other car given the start and ending point
# Things to keep in mind as we design the environment
# Where and how to spawn a car
# Where and how to spawn an obstacle car
# how to set a goal position 


# State :
# ego car: speed, distance to the goal, distance to the target car, speed of the target car.

# Reward :
# maintain a safe a distance between a ego car and target car
# earliest time to reach the goal position

# Action:
# throttle : -0.5 0 0.5
# steering angle : 0.5 0 0.5
# brake : (see if there is a need for a brake or not)

import gym
from gym import spaces
import numpy as np 
from custom_env.envs.testenv import Pygame2D

class CustomEnv(gym.Env):
	def __init__(self):
		self.pygame = Pygame2D()
		self.action_space = spaces.Discrete(3)
		self.observation_space = spaces.Box(np.array([0,0,0,0,0]), np.array([10,10,10,10,10]), dtype = np.int)

	def reset(self):
		del self.pygame 
		self.pygame = Pygame2D()
		obs =self.pygame.observe()
		return obs

	def step(self, action):
		self.pygame.action(action)
		obs = self.pygame.observe()
		reward = self.pygame.evaluate()
		done = self.pygame.is_done()
		return obs, reward, done, {}

	def render(self, mode = "human", close = False):
		self.pygame.view()
