# DQN Agent with gym env

import sys
import numpy as np
import math
import random

# implement the car to stay stationary for like 5 seconds when the car is like 100 seconds from the car


# Importing ENV
import gym
import custom_env

# DL imports 

from collections import deque
import keras
from tensorflow.keras.models import Model, load_model
from tensorflow.keras.layers import Input, Dense
from tensorflow.keras.optimizers import Adam, RMSprop

def OurModel(input_shape, action_space):
	x_input = Input(input_shape)
	x = x_input

	x = Dense(512, input_shape = input_shape, activation = "relu", kernel_initializer = 'he_uniform')(x)

	x = Dense(256, activation = "relu", kernel_initializer = 'he_uniform')(x)

	x = Dense(64, activation = "relu", kernel_initializer = 'he_uniform')(x)

	x = Dense(action_space, activation = "linear", kernel_initializer = 'he_uniform')(x)

	model = Model(inputs = x_input, outputs = x, name = 'CAR_DQN')
	model.compile(loss = "mse", optimizer = RMSprop(lr = 0.00025, rho = 0.95, epsilon = 0.01), metrics = ["accuracy"])

	model.summary()
	return model

class DQNAgent:
    def __init__(self):
        self.env = gym.make("Lanechange-v0")
        self.state_size = self.env.observation_space.shape[0]
        print("state_size:", self.state_size)
        self.action_size = self.env.action_space.n
        self.EPISODES = 9999
        self.MAX_TRY = 1000
        self.memory = deque(maxlen=2000)
        
        self.gamma = 0.6    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.001
        self.epsilon_decay = 0.999
        self.batch_size = 64
        self.train_start = 1000

        # create main model
        self.model = OurModel(input_shape=(self.state_size,), action_space = self.action_size)

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        if len(self.memory) > self.train_start:
            if self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay

    def act(self, state):
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            return np.argmax(self.model.predict(state))

    def replay(self):
        if len(self.memory) < self.train_start:
            return
        # Randomly sample minibatch from the memory
        minibatch = random.sample(self.memory, min(len(self.memory), self.batch_size))

        state = np.zeros((self.batch_size, self.state_size))
        next_state = np.zeros((self.batch_size, self.state_size))
        action, reward, done = [], [], []

        # do this before prediction
        # for speedup, this could be done on the tensor level
        # but easier to understand using a loop
        for i in range(self.batch_size):
            state[i] = minibatch[i][0]
            action.append(minibatch[i][1])
            reward.append(minibatch[i][2])
            next_state[i] = minibatch[i][3]
            done.append(minibatch[i][4])

        # do batch prediction to save speed
        target = self.model.predict(state)
        target_next = self.model.predict(next_state)

        for i in range(self.batch_size):
            # correction on the Q value for the action used
            if done[i]:
                target[i][action[i]] = reward[i]
            else:
                # Standard - DQN
                # DQN chooses the max Q value among next actions
                # selection and evaluation of action is on the target Q Network
                # Q_max = max_a' Q_target(s', a')
                target[i][action[i]] = reward[i] + self.gamma * (np.amax(target_next[i]))

        # Train the Neural Network with batches
        self.model.fit(state, target, batch_size=self.batch_size, verbose=0)


    def load(self, name):
        self.model = load_model(name)

    def save(self, name):
        self.model.save(name)
            
    def run(self):
        for e in range(self.EPISODES):
            state = self.env.reset()
            #print("state before reshape:", state)
            state = np.reshape(state, [1, self.state_size])
            #print("state AFTER reshape: ", state)
            done = False
            total_reward = 0

            for t in range(self.MAX_TRY):
            	action = self.act(state)
            	next_state, reward, done, _ = self.env.step(action)
            	total_reward += reward
            	next_state = np.reshape(next_state, [1, self.state_size])
            	self.remember(state, action, reward, next_state, done)
            	state = next_state
            	
            	# Rendering environment
            	self.env.render()
            	if (done) or (t >= self.MAX_TRY - 1):
            		print("Episode %d finished after %i time steps with total reward = %f." % (e, t, total_reward))
            		break

            	self.replay()

                # When episode is done, print reward


'''
    # To visvaluise the results
    pylab.figure(figsize =(18, 9))
	def PlotModel(self, score, episode):
		self.scores.append(score)
		self.episodes.append(episode)
		self.average.append(sum(self.scores)/len(self.scores))
		pylab.plot(self.episodes, self.average, 'r')
		pylab.plot(self.episodes, self.scores, 'b')
		pylab.ylabel('Score', fontsize = 18)
		pylab.xlabel('Steps', fontsize = 18)
		dqn = 'DQN_'
		Softupdate = ''
		if self.ddqn:
			dqn = 'DDQN_'
		if self.Soft_Update:
			Softupdate = '_soft'
		try:
			pylab.savefig(dqn + self.env_name + softupdate + ".png")
		except OSError:
			pass


		return str(self.average[-1])[:5]
'''


if __name__ == "__main__":
    agent = DQNAgent()
    agent.run()
    #agent.test()