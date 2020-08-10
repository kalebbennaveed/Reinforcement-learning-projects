import pygame
import sys
import math

pygame.init()
screen = pygame.display.set_mode((720, 266))
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
pos = (10, 100)
center = [pos[0] + 15, pos[1] + 19]
angle = 0

def check_radar(degree):
    len = 0
    x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
    y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)
    #print(center)
    #print(x)
    #print(y)

    while not bg.get_at((x, y)) == (255, 255, 255, 255) and len < 100:
        len = len + 1
        x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
        y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)
        #print(x)
        #print(y)
        #print("--------")
    #print("aafter computing", x, y)

    dist = int(math.sqrt(math.pow(x - center[0], 2) + math.pow(y - center[1], 2)))
    radars.append([(x, y), dist])

def check_radar_for_draw(degree):
    len = 0
    x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
    y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)

    while not bg.get_at((x, y)) == (255, 255, 255, 255) and len < 100:
        len = len + 1
        x = int(center[0] + math.cos(math.radians(360 - (angle + degree))) * len)
        y = int(center[1] + math.sin(math.radians(360 - (angle + degree))) * len)

    dist = int(math.sqrt(math.pow(x - center[0], 2) + math.pow(y - center[1], 2)))
    radars_for_draw.append([(x, y), dist])

def draw_radar(screen):    
    for r in radars_for_draw:
        pos, dist = r
        pygame.draw.line(screen, (0, 255, 0), center, pos, 1)
        pygame.draw.circle(screen, (0, 255, 0), pos, 2)


def draw_collision(screen):
    for i in range(4):
        x = int(four_points[i][0])
        y = int(four_points[i][1])
        pygame.draw.circle(screen, (255, 255, 255), (x,y), 2)

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
    #pygame.draw.circle(bg, (255, 255, 0), (25, 119), 15, 1)
    pygame.draw.rect(bg, (0, 0, 255), (368, 110, 38, 20), 7)
    #pygame.draw.rectangle(bg, (255, 255, 0), (670, 130), 15, )
    print(bg.get_at([25, 119]))
    for d in range(-90, 120, 30):
        check_radar(d)

    for d in range(-90, 120, 30):
        check_radar_for_draw(d)

    draw_radar(screen)
    # drawing collision points 
    center = [int(pos[0]) + 17, int(pos[1]) + 19]
    len = 17
    left_top = [center[0] + math.cos(math.radians(360 - (angle + 30))) * len, center[1] + math.sin(math.radians(360 - (angle + 30))) * len]
    right_top = [center[0] + math.cos(math.radians(360 - (angle + 150))) * len, center[1] + math.sin(math.radians(360 - (angle + 150))) * len]
    left_bottom = [center[0] + math.cos(math.radians(360 - (angle + 210))) * len, center[1] + math.sin(math.radians(360 - (angle + 210))) * len]
    right_bottom = [center[0] + math.cos(math.radians(360 - (angle + 330))) * len, center[1] + math.sin(math.radians(360 - (angle + 330))) * len]
    print("left top", left_top)
    print("right_top", right_top)
    print("left_bottom", left_bottom)
    print("right_bottom", right_bottom)
    four_points = [left_top, right_top, left_bottom, right_bottom]

    draw_collision(screen)    

    #print(pos1.colliderect(pos2))
    #screen.blit(player, (300, 190))
    #screen.blit(player,(i,i))
    pygame.display.update()