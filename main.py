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
	# Read two parameters identifying modules for the first and the second robots.
	if (len(sys.argv) < 3):
		print "Usage: python main.py <first_robot> <second_robot> [random seed]"
		print ""
		print "The <first_robot> and <second_robot> should identify modules containing classes Robot and RobotServer"
		print "E.g if you invoke "
		print "  python main.py telliskivi ekrs"
		print "The simulator will import telliskivi.Robot, telliskivi.RobotServer, ekrs.Robot, ekrs.RobotServer"
		sys.exit(1)
	
	# Try to import modules
	r1module = __import__(sys.argv[1])
	r2module = __import__(sys.argv[2])
	(a,b,c,d) = (r1module.Robot, r1module.RobotServer, r2module.Robot, r2module.RobotServer) # Testing
	random_seed = int(sys.argv[3]) if len(sys.argv) > 3 else None
	# random seeds 1,2,3,4 are already interesting use cases
	
	# Init graphics
	pygame.init()
	window = pygame.display.set_mode((1060, 760)) # This is the size of the field + contestant area. (5300 x 3800)
	pygame.display.set_caption('Robotex 2011 Simulator') 
	screen = pygame.display.get_surface() 

	# Init world. 
	world = World(screen)

	# Add 11 balls (coordinates are world-coords)
	# Make sure the balls are added symmetrically. That means the first ball goes in the center
	world.add_object(Ball(Point(world.width/2, world.height/2)))	
	for i in range(5):
		while True:
			xpos = random.uniform(10,world.width-10)
			ypos = random.uniform(10,world.height-10)
			# Make sure the positions do not get in the robot's starting corners ( 0..60px, i.e. 0..60px )
			if not ((xpos < 60 and ypos < 60) or (xpos > world.width - 60 and ypos > world.height - 60)):
				break
		world.add_object(Ball(Point(xpos, ypos)))
		world.add_object(Ball(Point(world.width-xpos, world.height-ypos)))
	
	# Create two robots
	robot1 = r1module.Robot(world, "Robot A", "TOPLEFT")
	robot2 = r2module.Robot(world, "Robot B", "BOTTOMRIGHT")
	world.add_object(robot1)
	world.add_object(robot2)
	
	# Start robot command servers
	r1module.RobotServer(robot1, 5000).serve()
	r2module.RobotServer(robot2, 5001).serve()
	
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