import pygame,sys,random
from pygame.locals import *
from pygame.time import get_ticks

from world import Point, World, Ball
from robot import Robot, RobotServer

# ---------------- Main program logic ----------------
def input(events): 
	for event in events: 
		if event.type == QUIT: 
			sys.exit(0) 
		elif event.type == KEYDOWN:
			if event.key == K_ESCAPE:
				sys.exit(0)
		else: 
			pass #print event

def main():
	# Init graphics
	pygame.init()
	window = pygame.display.set_mode((1060, 760)) # This is the size of the field + contestant area. (5300 x 3800)
	pygame.display.set_caption('Robotex 2011 Simulator') 
	screen = pygame.display.get_surface() 
	
	# Init world. 
	world = World(screen)

	# Add 11 balls (coordinates are world-coords)
	#random.seed(3)
	for i in range(11):
		world.add_object(Ball(Point(random.uniform(10,world.width-10),random.uniform(10,world.height-10))))
	
	# Create two robots
	robot1 = Robot(world, "Robot A", Point(12+45/2, 12+35/2), beacon_point = Point(world.width + 50, world.cy))
	robot1.rotate(3.1415/2)
	robot2 = Robot(world, "Robot B", Point(world.width-(12+45/2), world.height-(12+35/2)),  beacon_point = Point(-50, world.cy))
	robot2.rotate(-3.1415/2)
	world.add_object(robot1)
	world.add_object(robot2)
		
	# Start robot command servers
	RobotServer(robot1, 5000).serve()
	RobotServer(robot2, 5001).serve()
	
	# Do the simulation/drawing/event cycle
	last_sim = -1000
	last_draw = -1000
	while True: 
		t = get_ticks()
		if (t - last_sim) > 1:
			# Simulate world (~1 iteration once every millisecond or so)
			# NB: This is kinda hard-coded into the logic currently,
			# i.e. World.simulate() and Ball.simulate() and anyone else is 
			# free to assume that a simulation step is 1ms. In particular,
			# the ball computes it's friction coefficient like that.
			world.simulate()
			last_sim = t
		
		if (t - last_draw) > 40:
			# Draw a frame once every 40 milliseconds or so (~25 fps)
			BACKGROUND_BLUE = (120,119,253)
			screen.fill(BACKGROUND_BLUE)
			world.draw(screen)
			pygame.display.flip()
			last_draw = t
		
		# Process input
		input(pygame.event.get()) 

if __name__ == "__main__":
	main()