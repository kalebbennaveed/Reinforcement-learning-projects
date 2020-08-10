# for the first kind of hDQN.
# Set the speed of the car zero for 5 seconds 
# There is a problem for that awell. 
# lets suppose the car stops when it is 100 m away from the the car. It stops for 5 seconds 

import pygame
import sys 
import math

screen_width = 720
screen_height = 266
check_point = (600, 148)
bounding_box = (370, 110)

class Car:
	def __init__(self, car_file, obstacle_file, road_file, pos, pos2):
		# Ego car
		self.ego = pygame.image.load(car_file)
		self.ego = pygame.transform.scale(self.ego, (36, 36))
		self.rotate_ego = self.ego
		#Target Car
		self.target = pygame.image.load(obstacle_file)
		self.target = pygame.transform.scale(self.target, (36, 18))
		# Road 
		self.road = pygame.image.load(road_file)
		# Other declarations
		self.egotar = False
		self.egotar_times = 0

		self.pos = pos
		self.pos2 = pos2
		self.center =[self.pos[0] + 15, self.pos[1] + 19]
		self.angle = 0
		self.speed = 0
		self.target_angle = 0 # the target vehicle is always facing straight.
		self.egotar_dist = 0
		self.goal_dist = 0
		self.lane_change = 0

		# if the car_ego and car_target are having the same heading angles AND ego_car is approx 100 meters from the target_car
		# then do the lane change
		# Substarct 20 degree angle to the car to give it the new heading angle
		# speed can be increase by the magitute of 2, which is normal.
		# Execute the maneuver for 3 seconds. For time we will use iterations of the loop
		# After add 20 degrees back to the heading angle of the the car to it the previous position
		 


		# Radar
		self.radars = []
		self.radars_for_draw = []
		self.is_alive = True
		# check and goal
		# -----------
		self.current_check = 0 # **************************************************************
		self.prev_distance = 0
		self.cur_distance = 0
		self.goal = False
		self.check_flag = False # **************************************************************
		self.distance = 0
		self.time_spent = 0

		for d in range(-90, 120, 45):
		    self.check_radar(d)

		for d in range(-90, 120, 45):
		    self.check_radar_for_draw(d)

	def draw(self, screen):
		screen.blit(self.rotate_ego, self.pos)
		screen.blit(self.target, self.pos2)

	def draw_collision(self, screen):
	    for i in range(4):
	        x = int(self.four_points[i][0])
	        y = int(self.four_points[i][1])
	        pygame.draw.circle(screen, (255, 255, 255), (x, y), 3)	

	def check_radar(self, degree):
	    len = 0
	    x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * len)
	    y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * len)

	    while not self.road.get_at((x, y)) == ((255, 255, 255, 255) or (0, 0, 255, 255)) and len < 200:
	        len = len + 0.5
	        x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * len)
	        y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * len)
	        #print(x)
	        #print(y)
	        #print("--------")
	    #print("after computing", x, y)

	    dist = int(math.sqrt(math.pow(x - self.center[0], 2) + math.pow(y - self.center[1], 2)))
	    self.radars.append([(x, y), dist])

	def check_radar_for_draw(self, degree):
	    len = 0
	    x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * len)
	    y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * len)

	    while not self.road.get_at((x, y)) == ((255, 255, 255, 255) or (0, 0, 255, 255)) and len < 200:
	        len = len + 0.5
	        x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * len)
	        y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * len)

	    dist = int(math.sqrt(math.pow(x - self.center[0], 2) + math.pow(y - self.center[1], 2)))
	    self.radars_for_draw.append([(x, y), dist])

	def draw_radar(self, screen):    
	    for r in self.radars_for_draw:
	        pos, dist = r
	        pygame.draw.line(screen, (0, 255, 0), self.center, pos, 1)
	        pygame.draw.circle(screen, (0, 255, 0), pos, 2)

	def check_collision(self):
	    self.is_alive = True
	    for p in self.four_points:
	        if self.road.get_at((int(p[0]), int(p[1]))) == (255, 255, 255, 255):
	            self.is_alive = False
	            break
	    #CHeck for collision for green colour car with blue box
	    for q in self.four_points:
	      	if self.road.get_at((int(q[0]), int(q[0]))) == (0, 0, 255, 255):
	       		self.is_alive = False
	       		break

	def check_middleradar(self):
		self.egotar = False
		for d in range(-90, 120, 45):
			self.check_radar(d)
		radars = self.radars
		middle_list = radars[2]
		middle_length = middle_list[1]
		#print("middle_radar", middle_list)
		#print("middle_radar", middle_length)
		if middle_length <= 100:
			if self.egotar_times == 0:
				self.egotar = True
				self.egotar_times = self.egotar_times + 1
			else:
				self.egotar = False
		else:
			self.egotar = False



	"""
	def check_targetcar(self):
		self.egotar = False
		p = bounding_box
		self.egotar_dist = get_distance(p, self.center)
		if (self.egotar_dist < 80 and ((self.target_angle - 5) <= self.angle <= (self.target_angle + 5))):
			if self.egotar_times == 0:	
				self.egotar = True
				self.egotar_times = self.egotar_times + 1
			else:
				self.egotar = False
		else:
			self.egoatar = False
	"""

	def check_checkpoint(self):
		p = check_point
		self.prev_distance = self.cur_distance
		self.goal_dist = get_distance(p, self.center)
		#print("distance", dist)
		if self.goal_dist < 70:
			#remove this sentence
			self.current_check += 1
			self.prev_distance = 9999
			self.check_flag = True
			self.goal = True
		else:
			self.goal = False
			#if self.current_check >= len(check_point):
			#    self.current_check = 0
			#    self.goal = True
			#else:
			#    self.goal = False
		self.cur_distance = self.goal_dist


	def get_distance(self):
		p = check_point
		dist = get_distance(p, self.center)
		return dist


	def update(self):
	    #check speed
	    self.speed -= 0.5
	    if self.speed > 10:
	        self.speed = 10
	    if self.speed < 1:
	        self.speed = 1


	    #check position
	    self.rotate_ego = rot_center(self.ego, self.angle)
	    self.pos[0] += math.cos(math.radians(360 - self.angle)) * self.speed
	    if self.pos[0] < 20:
	        self.pos[0] = 20
	    elif self.pos[0] > screen_width - 120:
	        self.pos[0] = screen_width - 120

	    self.distance += self.speed
	    self.time_spent += 1
	    self.pos[1] += math.sin(math.radians(360 - self.angle)) * self.speed
	    if self.pos[1] < 20:
	        self.pos[1] = 20
	    elif self.pos[1] > screen_height - 120:
	        self.pos[1] = screen_height - 120

	    # caculate 4 collision points
	    self.center = [int(self.pos[0]) + 17, int(self.pos[1]) + 19]
	    len = 17
	    left_top = [self.center[0] + math.cos(math.radians(360 - (self.angle + 30))) * len, self.center[1] + math.sin(math.radians(360 - (self.angle + 30))) * len]
	    right_top = [self.center[0] + math.cos(math.radians(360 - (self.angle + 150))) * len, self.center[1] + math.sin(math.radians(360 - (self.angle + 150))) * len]
	    left_bottom = [self.center[0] + math.cos(math.radians(360 - (self.angle + 210))) * len, self.center[1] + math.sin(math.radians(360 - (self.angle + 210))) * len]
	    right_bottom = [self.center[0] + math.cos(math.radians(360 - (self.angle + 330))) * len, self.center[1] + math.sin(math.radians(360 - (self.angle + 330))) * len]
	    self.four_points = [left_top, right_top, left_bottom, right_bottom]

class Pygame2D:
	def __init__(self):
		pygame.init()
		self.screen = pygame.display.set_mode((720 ,266))
		self.clock = pygame.time.Clock()
		self.car = Car('car.png', 'obstacle.png', 'longer_road.png', [10, 100], [370, 110])
		self.game_speed = 60
		self.mode = 0

		# if the car_ego and car_target are having the same heading angles AND ego_car is approx 100 meters from the target_car
		# then do the lane change
		# Substarct 20 degree angle to the car to give it the new heading angle
		# speed can be increase by the magitute of 2, which is normal.
		# Execute the maneuver for 3 seconds. For time we will use iterations of the loop
		# After add 20 degrees back to the heading angle of the the car to it the previous position

	'''
	def action(self, action):
		if action == 0:
			self.car.speed += 2
		if action == 1:
			self.car.angle += 5
		elif action == 2:
			self.car.angle -= 5

		self.car.update()
		self.car.check_collision()
		self.car.check_checkpoint()
		self.car.radars.clear()

		self.car.radars.clear()
		for d in range(-90, 120, 45):
			self.car.check_radar(d)

	'''
	def action(self, action):
		self.car.check_middleradar()
		if self.car.egotar:
			# rule based policy
			self.car.angle  -= 20
			self.car.speed = 8
			for t in range(6):
				print("Executing rule based planner........")
				#print("speed", self.car.speed)
				self.car.update()
				self.car.check_checkpoint()
				self.car.check_collision()

				self.car.radars.clear()
				for d in range(-90, 120, 45):
					self.car.check_radar(d)


			self.car.angle = self.car.angle + 15
			self.car.lane_change += 1
		# Reinforcent learning based policy
		else:
			#print("Implementing policy.....")
			#print("speed", self.car.speed)
			if action == 0:
				self.car.speed += 2
			if action == 1:
				self.car.angle += 5
			elif action == 2:
				self.car.angle -= 5

			self.car.update()
			self.car.check_collision()
			self.car.check_checkpoint()

			self.car.radars.clear()
			for d in range(-90, 120, 45):
				self.car.check_radar(d)

			
	def evaluate(self):
		reward = 0 
		if abs(self.car.angle) <= 10:
			reward = reward + 100

		#dist = self.car.get_distance()
		#print("distance", dist)
		if self.car.check_collision():
			reward -= 5000
		#if not self.car.is_alive:
		#	reward = reward + (1000 - dist)

		if not self.car.is_alive:
			reward = reward -10000 + self.car.distance
			if self.car.lane_change == 1:
				reward = reward + 2000
		if self.car.goal:
			reward = reward + 10000

		'''
	    reward = -2000
	    # reward function
	    # designing the reward function
	    if self.check_collision():
	    	reward += -1000
	    if not self.car.is_alive:
	        #reward = -10000 + self.car.distance
	        reward += (5000 - self.car.get_distance())
	    if self.car.goal:
	        reward += 3000
	    '''
		return reward

	def is_done(self):
	    if not self.car.is_alive or self.car.goal:
	        self.car.current_check = 0 # ********************************************
	        self.car.distance = 0
	        return True
	    return False

	def observe(self):
	    # return state
	    radars = self.car.radars

	    ret = [0, 0, 0, 0, 0]
	    for i, r in enumerate(radars):
	        ret[i] = int(r[1] / 20)

	    #rad = tuple(ret)
	    #obs = [rad, speed, distance_egotar, distance_goal]

	    return tuple(ret)

	def view(self):
		# Draw Agents
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				sys.exit()

		self.screen.blit(self.car.road, (0,0))
		self.car.draw(self.screen)
		#self.car.draw(self.screen)
		pygame.draw.circle(self.car.road, (255, 255, 0), check_point, 15, 1)
		pygame.draw.rect(self.car.road, (255, 255, 255), (368, 110, 38, 20), 4)
		
		self.car.radars_for_draw.clear()
		for d in range(-90, 120, 45):
			self.car.check_radar_for_draw(d)
		'''
		# Drawing radars
		for d in range(-90, 120, 45):
			self.car.check_radar(d)

		for d in range(-90, 120, 45):
			self.car.check_radar_for_draw(d)
		'''
		self.car.draw_radar(self.screen)	

		# Drawing collisions
		self.car.draw_collision(self.screen)

		pygame.display.update()
		self.clock.tick(self.game_speed)

def get_distance(p1, p2):
	return math.sqrt(math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2))

def rot_center(image, angle):
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image		

if __name__ == "__main__":
	obj = Pygame2D()
	while True:
		obj.view()
	
'''
    screen.blit(bg,(0,0))
    screen.blit(player1, (10, 100)) # Starting point # WILL BE DYNAMIC
    #screen.blit(player1, (640, 277)) # Goal point
    screen.blit(player2, (370, 110)) # Obstacle point # WILL BE ******STATIC
    pygame.draw.circle(bg, (255, 255, 0), (690, 148), 15, 1)
    pygame.draw.circle(bg, (255, 255, 0), (25, 119), 15, 1)
    pygame.draw.rect(bg, (0, 255, 0), (368, 110, 38, 20), 7)
    #pygame.draw.rectangle(bg, (255, 255, 0), (670, 130), 15, )
    print(bg.get_at([25, 120]))
    for d in range(-90, 120, 45):
        check_radar(d)

    for d in range(-90, 120, 45):
        check_radar_for_draw(d)

    draw_radar(screen)
    #print(pos1.colliderect(pos2))
    #screen.blit(player, (300, 190))
    #screen.blit(player,(i,i))
    pygame.display.update()
'''

'''
player1 = pygame.image.load('car.png')
player1 = pygame.transform.scale(player1, (36, 36))
player2 = pygame.image.load('obstacle.png')
player2 = pygame.transform.scale(player2, (36, 18))
bg = pygame.image.load('longer_road.png')
pos1 = player1.get_rect()
pos2 = player2.get_rect()
print("position for palyer1: ", pos1)
print("position for player2: ", pos2)
radars = []
radars_for_draw = []
center = [25 , 119]
angle = 0

def check_radar(degree):
    len = 0
    x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
    y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)
    print(center)
    print(x)
    print(y)

    while not bg.get_at((x, y)) == (255, 255, 255, 255) and len < 300:
        len = len + 1
        x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
        y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)
        #print(x)
        #print(y)
        #print("--------")
    print("aafter computing", x, y)

    dist = int(math.sqrt(math.pow(x - center[0], 2) + math.pow(y - center[1], 2)))
    radars.append([(x, y), dist])

def check_radar_for_draw(degree):
    len = 0
    x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
    y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)

    while not bg.get_at((x, y)) == (255, 255, 255, 255) and len < 300:
        len = len + 1
        x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
        y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)

    dist = int(math.sqrt(math.pow(x - center[0], 2) + math.pow(y - center[1], 2)))
    radars_for_draw.append([(x, y), dist])

def draw_radar(screen):    
    for r in radars_for_draw:
        pos, dist = r
        pygame.draw.line(screen, (0, 255, 0), center, pos, 1)
        pygame.draw.circle(screen, (0, 255, 0), pos, 5)

#pygame.draw.circle(screen, (255, 255, 255), (640, 277), 70, 10)

while True:
    for i in pygame.event.get():
       if i.type == pygame.QUIT:
            sys.exit()
    screen.blit(bg,(0,0))
    screen.blit(player1, (10, 100)) # Starting point # WILL BE DYNAMIC
    #screen.blit(player1, (640, 277)) # Goal point
    screen.blit(player2, (370, 110)) # Obstacle point # WILL BE ******STATIC
    pygame.draw.circle(bg, (255, 255, 0), (690, 148), 15, 1)
    pygame.draw.circle(bg, (255, 255, 0), (25, 119), 15, 1)
    pygame.draw.rect(bg, (0, 255, 0), (368, 110, 38, 20), 7)
    #pygame.draw.rectangle(bg, (255, 255, 0), (670, 130), 15, )
    print(bg.get_at([25, 120]))
    for d in range(-90, 120, 45):
        check_radar(d)

    for d in range(-90, 120, 45):
        check_radar_for_draw(d)

    draw_radar(screen)
    #print(pos1.colliderect(pos2))
    #screen.blit(player, (300, 190))
    #screen.blit(player,(i,i))
    pygame.display.update()
'''
'''
pygame.init()
screen = pygame.display.set_mode((720 ,560))
player1 = pygame.image.load('car.png')
player1 = pygame.transform.scale(player1, (100, 100))
player2 = pygame.image.load('obstacle.png')
player2 = pygame.transform.scale(player2, (100, 50))
bg = pygame.image.load('road.jpg')
pos1 = player1.get_rect()
pos2 = player2.get_rect()

while True:
    for i in pygame.event.get():
       if i.type == pygame.QUIT:
            sys.exit()
    screen.blit(bg,(0,200))
    screen.blit(player1, (620, 260)) # Goal point
    screen.blit(player2, (340, 210)) # Obstacle point
    #screen.blit(player, (300, 190))
    #screen.blit(player,(i,i))
    pygame.display.update()

'''
