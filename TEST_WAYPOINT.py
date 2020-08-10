
from __future__ import print_function
# NORMAL FUNCTION IMPORTS

import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import math
from collections import deque
from tqdm import tqdm

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- add PythonAPI for release mode --------------------------------------------
# These lines of code are important if you want to import other scripts
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
#from carla import ColorConverter as cc
#from agents.navigation.basic_agent import BasicAgent # Like this script
#from agents.navigation.local_planner import LocalPlanner
#from agents.navigation.global_route_planner import GlobalRoutePlanner
#from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


# TENSORFLOW 
import keras
from tensorflow.keras.applications.xception import Xception
from tensorflow.keras.models import Model, load_model
from tensorflow.keras.layers import Dense, GlobalAveragePooling2D, Input, Add, Lambda
from tensorflow.keras.optimizers import Adam, RMSprop
#from tensorflow.keras.callbacks import TensorBoard
import tensorflow as tf
from tensorflow.keras import backend as K
from threading import Thread


# Other useful planner imports 

# STATE VECTOR consists of this 
# ***FIRST STAGE***
# Velocity of the ego vehcile
# Velocity of the target vehcile
# acceleration of the ego car**
# jerk of the ego car**
# distance from the ego car
# distance from the goal position

# GLOBAL VARIABLES
# =========================================
SHOW_PREVIEW = False
IM_WIDTH = 640
IM_HEIGHT = 480
SECONDS_PER_EPISODE = 30
REPLAY_MEMORY_SIZE = 5_000
MIN_REWARD = -200
EPISODES = 3
AGGREGATE_STATS_EVERY = 10
FPS = 60
# ========================================


# This import and code prevents "prediction function callback" error
import tensorflow as tf
physical_devices = tf.config.list_physical_devices('GPU') 
tf.config.experimental.set_memory_growth(physical_devices[0], True)

# ====================================================
# BLACKBOARD AREA
# ====================================================
## Results right now
# The car has learned to stay in the lane with the reward of the

# The command to set frequency is: "carlaUE4 -benchmark -fps=10

# LSTM Layer
# Need a sequence of 20 steps fed into the model with single outpiut. T
# The dimensions of the input will be (4, 20 , 4)
# The dimendion of the output will be single layer



## PROBLEMS SECTION ##
# The car should get higher reward if it goes near the end point
# The car turns in the round abput (Already solved with the help of the )
# No use of global planner so far which has to be there

# so for lane change sceraios i am using basic agent in python scripts but it stucks idk where
# Probably need to get a function that can be draw a polinomial from car position to the middle of the 

## Expected behaviour ##
# Car should stay in a lane and then do a lane change manuever

## Rewards ##
# Negative reward for no lane invasion
# Negative reward for collision
# Positive reward for moving towards the final distance
#  

## Env should have ##
# Two Cars
# Lane and Collision sensor
# Distance to the car and distance to the final position

## Brainstorm area ##
# get the difference between the values of egogoal and egotar distance b/w two states.
# Let car itself determine the safe distance from the obstacle to do the lane change manuever 


# =====================================================================
# CLASS ENVIRONEMENT "CarEnv" for lane follow and lane change manuever  
# =====================================================================



class CarEnv:
    SHOW_CAM = SHOW_PREVIEW
    STEER_AMT = 0.3
    im_width = IM_WIDTH
    im_height = IM_HEIGHT

    def __init__(self):
        self.client = carla.Client('127.0.0.1', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.debug = self.world.debug
        self.map = self.world.get_map()
        self.world.set_weather(getattr(carla.WeatherParameters, 'ClearNoon'))
        self.blueprint_library = self.world.get_blueprint_library()
        self.model_3 = self.blueprint_library.filter("model3")[0]



    def reset(self):
        self.collision_hist = []
        self.invasion_hist = []
        self.actor_list = []
        self.lane_counter = 0
        self.is_alive = True

        self.transform = carla.Transform(carla.Location(x=59.0, y=3.8, z=1.843102), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
        self.vehicle = self.world.spawn_actor(self.model_3, self.transform)
        self.controller = VehiclePIDController(self.vehicle)
        self.actor_list.append(self.vehicle)
        
        self.transform2 = carla.Transform(carla.Location(x=90.2, y=4.1, z=1.843102), carla.Rotation(pitch = 0.0000, yaw = 0.855823, roll = 0.00000))
        self.target_vehicle = self.world.spawn_actor(self.model_3, self.transform2)
        self.actor_list.append(self.target_vehicle)

        #self.transform3 = carla.Transform(carla.Location(x =59.0, y = 7.3, z = 1.84312), carla.Rotation(pitch = 0.00000, yaw = 0.855823, roll = 0.00000))
        #self.moving_vehicle = self.world.spawn_actor(self.model_3, self.transform3)
        #self.actor_list.append(self.moving_vehicle)

        
        self.rgb_cam = self.blueprint_library.find('sensor.camera.rgb')
        self.rgb_cam.set_attribute("image_size_x", f"{self.im_width}")
        self.rgb_cam.set_attribute("image_size_y", f"{self.im_height}")
        self.rgb_cam.set_attribute("fov", f"110")

        transform = carla.Transform(carla.Location(x=-5.5, z=2.8))
        self.sensor = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.vehicle)
        self.actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.process_img(data))

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        #self.target_vehicle.apply_control(carla.VehicleControl(throttle = 0.0, brake=0.0))
        time.sleep(4)

        lane_sensor = self.blueprint_library.find("sensor.other.lane_invasion")
        self.lane_sensor = self.world.spawn_actor(lane_sensor, transform, attach_to = self.vehicle)
        self.actor_list.append(self.lane_sensor)
        self.lane_sensor.listen(lambda event_lane: self.invasion_data(event_lane))

        colsensor = self.blueprint_library.find("sensor.other.collision")
        self.colsensor = self.world.spawn_actor(colsensor, transform, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        time.sleep(0.01)

        self.episode_start = time.time()
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        #self.moving_vehicle.apply_control(carla.VehicleControl(throttle = 0.0, brake  = 0.0))

        obs = self.observe()
        return obs

    def get_speed(self, vehicle):
        vel = vehicle.get_velocity()
        return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


    def observe(self):
        # State space include
        self.observe_vector = []

        #ego car velocity
        v = self.vehicle.get_velocity()
        ego_kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        self.observe_vector.append(ego_kmh)

        # Target car velocity
        v = self.target_vehicle.get_velocity()
        target_kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        self.observe_vector.append(target_kmh)

        # Distance between target and ego car
        dist_egotarget = self.ego_target()
        self.observe_vector.append(dist_egotarget)

        # Distance between ego and goal destination
        dist_egogoal = self.ego_goal()
        self.observe_vector.append(dist_egogoal)

        return tuple(self.observe_vector)

    def collision_data(self, event):
        self.collision_hist.append(event)

    def invasion_data(self,event_lane):
        self.invasion_hist.append(event_lane)

    def process_img(self, image):
        i = np.array(image.raw_data)
        #print(i.shape)
        i2 = i.reshape((self.im_height, self.im_width, 4))
        i3 = i2[:, :, :3]
        if self.SHOW_CAM:
            cv2.imshow("", i3)
            cv2.waitKey(1)
        #self.front_camera = i3

    def ego_target(self):
        # Distance between target and ego car
        self.dist_ET = carla.Location.distance(self.vehicle.get_location(),self.target_vehicle.get_location())
        return self.dist_ET

    def ego_goal(self):
        # Distance between ego and goal destination
        goal_position = carla.Transform(carla.Location(x=120.8, y=4.3, z=0), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
        dist_EG = carla.Location.distance(self.vehicle.get_location(), goal_position.location)
        return dist_EG

    '''
    def lane_change(self):
        ego_location = self.vehicle.get_location()
        ego_waypoint = self.map.get_waypoint(ego_location)
        # Right lane ego point
        lane_change = str(ego_waypoint.lane_change)
        if lane_change == 'Right':
            waypoint_rightlane = ego_waypoint.get_right_lane()
            # final point of lane change trajectory
            waypoint_final = waypoint_rightlane.next(10.0)[0]
        agent = BasicAgent(self.vehicle)
        agent.set_destination(waypoint_final)
        while True:
            control = agent.run_step()
        #lchange_time = 2
        #start_time = time.time()
        #while time.time() < start_time + lchange_time:    
            self.vehicle.apply_control(control)
    '''
    '''
    def lane_change(self):
        print("Going in Lane Change Option")
        ego_location = self.vehicle.get_location()
        ego_waypoint = self.map.get_waypoint(ego_location, project_to_road = True)
        # Right lane ego point
        target_speed = 20
        lane_change = str(ego_waypoint.lane_change)
        if lane_change == 'Right':
            waypoint_rightlane = ego_waypoint.get_right_lane()
            # final point of lane change trajectory
            waypoint_final = waypoint_rightlane.next(15.0)[0]
            lchange_time = 2.5
            start_time = time.time()
            while time.time() < start_time + lchange_time:
                control = self.controller.run_step(target_speed, waypoint_final)    
                self.vehicle.apply_control(control)
            final_location = self.vehicle.get_location()
            self.world.debug.draw_point(ego_location, size = 0.3, life_time = 5)
            self.world.debug.draw_point(final_location, size = 0.3, life_time = 5)
            self.world.debug.draw_line(ego_location, final_location, thickness = 0.3, life_time = 5)
            self.lane_counter += 1
            ego_waypoint2 = self.map.get_waypoint(self.vehicle.get_location(), project_to_road = True)
            waypoint_final2 = ego_waypoint2.next(13.0)[0]
            correct_time = 1.0
            start_time2 = time.time()
            while time.time() < start_time2 + correct_time:
                control = self.controller.run_step(target_speed, waypoint_final2)
                self.vehicle.apply_control(control)
            last_location = self.vehicle.get_location()
            self.world.debug.draw_point(last_location, size = 0.3, life_time = 3)            
            self.world.debug.draw_line(final_location, last_location, thickness = 0.3, life_time = 3)                
            print("Going out of Lane Change")
    '''
    def is_done(self):
        if self.is_alive == False:
            return True
        else:
            return False


    def step(self, action):

        ego_waypoint = self.map.get_waypoint(self.vehicle.get_location(), project_to_road = True)
        waypoint_rightlane = ego_waypoint.get_right_lane()
        if action == 0:
            final_waypoint = waypoint_rightlane.next(7.0)[0]
        elif action == 1:
            final_waypoint = waypoint_rightlane.next(14.0)[0]
        elif action == 2:
            final_waypoint = waypoint_rightlane.next(20.0)[0]

        target_speed = 20
        lchange_time = 2.5
        start_time = time.time()
        while time.time() < start_time + lchange_time:
            control = self.controller.run_step(target_speed, final_waypoint)    
            self.vehicle.apply_control(control)        
        final_location = self.vehicle.get_location()
        self.world.debug.draw_point(ego_location, size = 0.3, life_time = 5)
        self.world.debug.draw_point(final_location, size = 0.3, life_time = 5)
        self.world.debug.draw_line(ego_location, final_location, thickness = 0.3, life_time = 5)
        ego_waypoint2 = self.map.get_waypoint(self.vehicle.get_location(), project_to_road = True)
        waypoint_final2 = ego_waypoint2.next(13.0)[0]
        correct_time = 1.5
        start_time2 = time.time()
        while time.time() < start_time2 + correct_time:
            control = self.controller.run_step(target_speed, waypoint_final2)
            self.vehicle.apply_control(control)
        last_location = self.vehicle.get_location()
        self.world.debug.draw_point(last_location, size = 0.3, life_time = 3)            
        self.world.debug.draw_line(final_location, last_location, thickness = 0.3, life_time = 3)                
        print("Going out of Lane Change")


        v = self.vehicle.get_velocity()
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))

        reward = 0

        if self.ego_goal() <= 10:
            print("Goal Reached")
            self.is_alive = False
            reward += 1000

        #print(self.ego_goal())
        if self.lane_counter == 1:
            reward += 200

        if len(self.collision_hist) != 0:
            self.is_alive = False
            reward += -200

        if kmh > 50:
            reward += -60

        if kmh < 20:
            reward += -100

        #for event in self.invasion_hist:
        #    if event == 'Solid' or 'SolidSolid':
        #        done = True
        #        reward -= 500
        for event in self.invasion_hist:
            print("invasion_hist", event)
        if len(self.invasion_hist) != 0:
            reward += -50

        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            self.is_alive = False

        obs = self.observe()
        done = self.is_done()

        return obs, reward, done, None

# ===================================================
# CONTROLLER
#====================================================

class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, vehicle, args_lateral=None, args_longitudinal=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID controller using the following
        semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        if not args_lateral:
            args_lateral = {'K_P': 0.8, 'K_D': 0.01, 'K_I': 1.4}
        if not args_longitudinal:
            args_longitudinal = {'K_P': 0.8, 'K_D': 0.0, 'K_I': 1.0}

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
        self._lat_controller = PIDLateralController(self._vehicle, **args_lateral)

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """
        throttle = self._lon_controller.run_step(target_speed)
        steering = self._lat_controller.run_step(waypoint)

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)
        self.env_car = CarEnv()

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """
        current_speed = self.env_car.get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        _e = (target_speed - current_speed)
        self._e_buffer.append(_e)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt), 0.0, 1.0)


class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _dot) + (self._K_D * _de /
                                             self._dt) + (self._K_I * _ie * self._dt), -1.0, 1.0)

# ===================================================
# DQN Agent
# ===================================================

class DQNAgent:
    def __init__(self):
        self.state_size = 4
        self.action_size = 2
        self.EPISODES = 9999
        self.memory = deque(maxlen=REPLAY_MEMORY_SIZE)

        
        self.gamma = 0.6    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.001
        self.epsilon_decay = 0.999
        self.batch_size = 64
        self.train_start = 1000
        self.TAU = 0.1

        # create main model
        self.model = self.create_model(input_shape=(self.state_size,), action_space = self.action_size)
        self.target_model = self.create_model(input_shape=(self.state_size,), action_space = self.action_size)


    # things this network has
    # Dueling Double Q Networks
    # Soft Updates
    # Target Model 

    def create_model(self, input_shape, action_space):
        x_input = Input(input_shape)
        x = x_input
        x = Dense(512, input_shape = input_shape, activation ="relu", kernel_initializer = 'he_uniform')(x)
        x = Dense(256, activation = "relu", kernel_initializer = 'he_uniform')(x)
        x = Dense(64, activation = "relu", kernel_initializer = 'he_uniform')(x)
        x = Dense(action_space, activation = "linear", kernel_initializer = 'he_uniform')(x)

        # Dueling Structure
        state_value = Dense(1, kernel_initializer = 'he_uniform')(x)
        state_value = Lambda(lambda s: K.expand_dims(s[:, 0], -1), output_shape =(action_space,))(state_value)
        action_advantage = Dense(action_space, kernel_initializer = 'he_uniform')(x)
        action_advantage = Lambda(lambda a: a[:,:] - K.mean(a[:,:], keepdims = True), output_shape = (action_space,))(action_advantage)

        x = Add()([state_value, action_advantage])
        model = Model(inputs = x_input, outputs = x, name = 'Lane_Change')
        model.compile(loss = "mse", optimizer = RMSprop(lr = 0.00025, rho = 0.95, epsilon = 0.01), metrics = ["accuracy"])
        model.summary()
        return model

    def update_target_model(self):
        q_model_theta = self.model.get_weights()
        target_model_theta = self.target_model.get_weights()
        counter = 0
        for q_weight, target_weight in zip(q_model_theta, target_model_theta):
            target_weight = target_weight * (1-self.TAU) + q_weight * self.TAU
            target_model_theta[counter] = target_weight
            counter += 1
        self.target_model.set_weights(target_model_theta)


    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        if len(self.memory) > self.train_start:
            if self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay
    
    def act(self, state):   
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_size)
            time.sleep(1/FPS)
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

        for i in range(self.batch_size):
            state[i] = minibatch[i][0]
            action.append(minibatch[i][1])
            reward.append(minibatch[i][2])
            next_state[i] = minibatch[i][3]
            done.append(minibatch[i][4])

        # do batch prediction to save speed
        target = self.model.predict(state)
        target_next = self.model.predict(next_state)
        target_val = self.target_model.predict(next_state)

        for i in range(self.batch_size):
            # correction on the Q value for the action used
            if done[i]:
                target[i][action[i]] = reward[i]
            else:
                # Standard - DQN
                # DQN chooses the max Q value among next actions
                # selection and evaluation of action is on the target Q Network
                # Q_max = max_a' Q_target(s', a')
                # DDQN
                a = np.argmax(target_next[i])
                target[i][action[i]] = reward[i] + self.gamma * (target_val[i][a])

        # Train the Neural Network with batches
        self.model.fit(state, target, batch_size=self.batch_size, verbose=0)


    def load(self, name):
        self.model = load_model(name)

    def save(self, name):
        self.model.save(name)

if __name__ == '__main__':

    # For stats
    ep_rewards = [-200]

    # Create agent and environment
    agent = DQNAgent()
    env = CarEnv()

    # Iterate over episodes
    for episode in tqdm(range(1, EPISODES + 1), ascii=True, unit='episodes'):
        #try:

            env.collision_hist = []
            env.invasion_hist = []
            episode_reward = 0
            step = 1
            state = env.reset()

            state = np.reshape(state, [1, agent.state_size])

            done = False
            episode_start = time.time()


            while True:
                #env.moving_vehicle.apply_control(carla.VehicleControl(throttle = 0.8, brake = 0.0, steer = 0.0))
                action = agent.act(state)
                next_state, reward, done, _ = env.step(action)
                episode_reward += reward

                next_state = np.reshape(next_state, [1, agent.state_size])

                # Every step we update replay memory
                agent.remember(state, action, reward, next_state, done)
                state = next_state
                step += 1

                if done:
                    agent.update_target_model()
                    print("Episode %d finished with total reward = %f." % (episode, episode_reward))
                    break

                agent.replay()

            # End of episode - destroy agents
            for actor in env.actor_list:
                actor.destroy()

