import pygame,random
from pygame.locals import *
from pygame import draw
from math import sin, cos, sqrt

# -------------- Utility class -------------------
class Point:
	"""
	Basic 2-tuple with vector operations.
	 >>> p = Point(2, 0)
	 >>> p.norm()
	 2.0
	 >>> p.normalize()
	 >>> p
	 Point(1.000000, 0.000000)
	 >>> p.add(Point(0,2))
	 >>> p
	 Point(1.000000, 2.000000)
	 >>> p.mul(3)
	 >>> p
	 Point(3.000000, 6.000000)
	 >>> p.as_tuple()
	 (3, 6)
	 >>> Point(1, 2) + Point(2, 2)
	 Point(3.000000, 4.000000)
	 >>> Point(4, 1)*(-0.5)
	 Point(-2.000000, -0.500000)
	 >>> Point(-1,2).inner_product(Point(3,-1))
	 -5
	"""
	def __init__(self, x_or_xy, y=None):
		if (y is None):
			self.x, self.y = x_or_xy
		else:
			self.x, self.y = x_or_xy, y
	
	def __add__(self, p):
		return Point(self.x + p.x, self.y + p.y)
		
	def add(self, p):
		self.x, self.y = self.x + p.x, self.y + p.y
	
	def mul(self, c):
		self.x, self.y = self.x*c, self.y*c
	
	def __sub__(self, p):
		return Point(self.x - p.x, self.y - p.y)

	def __mul__(self, c):
		return Point(c*self.x, c*self.y)
	
	def norm(self):
		return sqrt(self.x*self.x + self.y*self.y)
		
	def normalize(self):
		n = self.norm()
		self.x, self.y = self.x/n, self.y/n

	def normalized(self):
		n = self.norm()
		return Point(self.x/n, self.y/n)
		
	def as_tuple(self):
		return (int(self.x), int(self.y))
	
	def rotate(self, angle):
		cosa, sina = cos(angle), sin(angle)
		self.x, self.y = self.x*cosa + self.y*sina, self.x*(-sina) + self.y*cosa
	
	def inner_product(self, v):
		return self.x*v.x + self.y*v.y
	
	def in_rect(self, bottomleft, topright):
		"""
		Returns true if the point is strictly within the rectangle.
		The bottomleft and topright correspond the (minx, miny) and (maxx, maxy) corners.
		"""
		return bottomleft.x < self.x and self.x < topright.x and bottomleft.y < self.y and self.y < topright.y
	
	def __repr__(self):
		return "Point(%f, %f)" % (self.x, self.y)

# -------------- Another utility class -------------------
class Wall:
	"""
	A Wall represents a segment with a normal pointing to the right (in the right-handed coordinates).
	It pre-stores the normal and its length to speed-up computations later on.
	>>> w = Wall(Point(1, 1), Point(10, 1))
	>>> w.len					# Length of the segment
	9.0
	>>> w.v						# Direction vector pointing from first to the second point
	Point(9.000000, 0.000000)
	>>> w.v_normalized			# Same thing, normalized to length one
	Point(1.000000, 0.000000)
	>>> w.normal				# Direction vector rotated to the right
	Point(0.000000, -1.000000)
	>>> w.dist_to_point(Point(3, 1))	# Point lying on the segment
	0.0
	>>> w.dist_to_point(Point(3, 0))	# Point lying to the right
	1.0
	>>> w.dist_to_point(Point(3, 2))	# Point lying to the left
	-1.0
	"""
	def __init__(self, p1, p2):
		self.p1 = p1
		self.p2 = p2
		self.v = p2 - p1
		self.len = self.v.norm()
		self.v_normalized = self.v * (1/self.len)
		self.normal = Point(self.v_normalized.y, -self.v_normalized.x) # The normal is a unit vector pointing "inside" the walled area

	def dist_to_point(self, pt):
		pp = pt - self.p1
		xprod = pp.x*self.v.y - pp.y*self.v.x
		return xprod/self.len
	
	def __repr__(self):
		return "Wall(%s, %s) of length %f" % (str(self.p1), str(self.p2), self.len)
	
	
# -------------- The world is the root controller for simulation and drawing -------------------

class World:
	"""
	The World represents the football field, where the robot and the balls live.
	It's main routines are:
		* add_object	- registers a new object with the world.
		* simulate		- perform a single simulation step. Typically about 50 steps should be done between frames.
		* draw			- render the world on a pygame surface.
	Here's how it goes typically
	>>> import pygame
	>>> from telliskivi import Robot
	>>> (_1, _2) = pygame.init()
	>>> window = pygame.display.set_mode((1060, 760)) # This is the recommended size of the window (field + contestant area - (5300 x 3800 mm))
	>>> screen = pygame.display.get_surface() 
	>>> w = World(screen)
	>>> w.width
	900
	>>> w.height
	600
	>>> w.add_object(Ball(Point(300, 300)))				# Add a ball at position (300, 300)
	>>> w.add_object(Robot(w, "Robot", "TOPLEFT"))      # Add a robot
	>>> for i in range(50):								# Simulate a bit
	...    w.simulate()
	>>> w.draw(screen)									# Draw on screen
	
	See also: WorldObject
	"""
	def __init__(self, screen):
		# Actual size of the field is 4500x3000. We make it 900x600 in pixels, which means each pixel is 5mm in reality
		self.width, self.height = 900, 600
		self.cx, self.cy = self.width/2, self.height/2
		sw, sh = screen.get_size()
		scx, scy = sw/2, sh/2
		left, top = scx - self.width/2, scy - self.height/2
		# Create a sub-image, containing the whole of the world in it.
		self.screen = screen
		self.font = pygame.font.Font(None, 60)
		self.scoreLeft = 0
		self.scoreRight = 0
		self.field = screen.subsurface(Rect(left, top, self.width, self.height))
		self.objects = [] # This will hold all the objects in the world
		# Walls listed in clockwise order (in the right-hand coords)
		self.walls = [Wall(Point(0,0), Point(0,self.height)), \
					  Wall(Point(0,self.height), Point(self.width, self.height)),\
					  Wall(Point(self.width, self.height), Point(self.width, 0)),\
					  Wall(Point(self.width, 0), Point(0, 0))]
	
	def draw(self, screen):
		# The field is a rect in the center of the screen
		BLACK = (0,0,0)
		WHITE = (255,255,255)
		GREEN = (43,252,43)
		self.field.fill(GREEN)	# Green fill
		draw.rect(self.field, WHITE, self.field.get_rect(), 20)	# White border
		draw.rect(self.field, BLACK, self.field.get_rect(), 1)		# Black border
		draw.rect(self.field, BLACK, (10, 10, self.cx-5-10, self.height-10-10), 1)	# Black square (left)
		draw.rect(self.field, BLACK, (self.cx + 5, 10, self.cx-5-10, self.height-10-10), 1)	# Black square (right)
		draw.circle(self.field, WHITE, (self.cx, self.cy), 80, 10)	# Central circle (white)
		draw.circle(self.field, BLACK, (self.cx, self.cy), 80-10, 1)		# Central circle (black, inner)
		draw.circle(self.field, BLACK, (self.cx, self.cy), 80, 1)			# Central circle (black, outer)
		draw.line(self.field, WHITE, (self.cx-1, 0+1), (self.cx-1, self.height-2),10)	# Central divider line (white)

		# Arcs
		#1
		draw.arc(self.field, WHITE, (10 - 100, 265 - 100, 200, 200), 0, 3.1415/2, 10)			# Left goal, upper arc, white filling
		draw.arc(self.field, BLACK, (10 - 100, 265 - 100, 200, 200), 0, 3.1415/2, 1)			# Left goal, upper arc, black, outer
		draw.arc(self.field, BLACK, (10 - 100 + 10, 265 - 100 + 10, 200 - 20, 200 - 20), 0, 3.1415/2, 1)	# Left goal, upper arc, black inner

		#2
		draw.arc(self.field, WHITE, (10 - 100, 265 - 100 + 70, 200, 200), 3*3.1415/2, 2*3.1415, 10)			# Left goal, lower arc, white filling
		draw.arc(self.field, BLACK, (10 - 100, 265 - 100 + 70, 200, 200), 3*3.1415/2, 2*3.1415, 1)			# Left goal, lower arc, black, outer
		draw.arc(self.field, BLACK, (10 - 100 + 10, 265 - 100 + 10 + 70, 200 - 20, 200 - 20), 3*3.1415/4, 2*3.1415, 1)	# Left goal, lower arc, black inner

		#3
		draw.arc(self.field, WHITE, (self.width - 10 - 100, 265 - 100, 200, 200), 3.1415/2, 3.1415, 10)			# Right goal, upper arc, white filling
		draw.arc(self.field, BLACK, (self.width - 10 - 100, 265 - 100, 200, 200), 3.1415/2, 3.1415,  1)			# Right goal, upper arc, black, outer
		draw.arc(self.field, BLACK, (self.width - 10 - 100 + 10, 265 - 100 + 10, 200 - 20, 200 - 20), 3.1415/2, 3.1415, 1)	# Left goal, upper arc, black inner

		#4
		draw.arc(self.field, WHITE, (self.width - 10 - 100, 265 - 100 + 70, 200, 200), 3.1415, 3*3.1415/2, 10)			# Right goal, lower arc, white filling
		draw.arc(self.field, BLACK, (self.width - 10 - 100, 265 - 100 + 70, 200, 200), 3.1415, 3*3.1415/2, 1)			# Right goal, lower arc, black, outer
		draw.arc(self.field, BLACK, (self.width - 10 - 100 + 10, 265 - 100 + 10 + 70, 200 - 20, 200 - 20),  3.1415, 3*3.1415/2, 1)	# Right goal, lower arc, black inner
		
		# Arc connectors
		draw.line(self.field, WHITE, (10 + 100 - 5, 265), (10 + 100 - 5, 265 + 70), 10) # Left goal, arc connector, white filling
		draw.line(self.field, BLACK, (10 + 100 - 10, 265), (10 + 100 - 10, 265 + 70), 1) # Left goal, arc connector, black inner
		draw.line(self.field, BLACK, (10 + 100, 265), (10 + 100, 265 + 70), 1) # Left goal, arc connector, black outer
		
		draw.line(self.field, WHITE, (self.width - 10 - 100 + 5, 265), (self.width - 10 - 100 + 5, 265 + 70), 10) # Right goal, arc connector, white filling
		draw.line(self.field, BLACK, (self.width - 10 - 100 + 10, 265), (self.width - 10 - 100 + 10, 265 + 70), 1) # Right goal, arc connector, black inner
		draw.line(self.field, BLACK, (self.width - 10 - 100, 265), (self.width - 10 - 100, 265 + 70), 1) # Right goal, arc connector, black outer
		
		# Left goal
		draw.rect(self.screen, (163, 163, 46), (self.field.get_offset()[0]-50, self.field.get_offset()[1]+self.cy-70, 50+10, 140), 0)
		text = self.font.render(str(self.scoreLeft), True, BLACK, (163, 163, 46))
		textRect = text.get_rect()
		# Center the rectangle
		textRect.centerx = self.field.get_offset()[0]-50 + 25 + 5
		textRect.centery = self.field.get_offset()[1]+self.cy
		# Blit the text
		self.screen.blit(text, textRect)
		
		# Right goal
		draw.rect(self.screen, (16, 57, 125), (self.field.get_offset()[0]+self.width-10, self.field.get_offset()[1]+self.cy-70, 50+10, 140), 0)
		text = self.font.render(str(self.scoreRight), True, WHITE, (16, 57, 125))
		textRect = text.get_rect()
		textRect.centerx = self.field.get_offset()[0]+self.width - 10 + 30
		textRect.centery = self.field.get_offset()[1]+self.cy
		self.screen.blit(text, textRect)
		
		# Cross in the middle
		#draw.line(self.field, BLACK, (self.cx-10, self.cy), (self.cx+10, self.cy))
		#draw.line(self.field, BLACK, (self.cx, self.cy-10), (self.cx, self.cy+10))
		
		for o in self.objects:
			o.draw(self.field)
		
	def add_object(self, obj):
		"""The world manages a set of objects. Each object must have particular properties"""
		self.objects.append(obj)
		
	def simulate(self):
		for o in self.objects:
			o.simulate()
		# Resolve collisions
		# First the walls
		for o in self.objects:
			for w in self.walls:
				o.wall_check(w)
		# Then the collision among the objects
		for i in range(len(self.objects)):
			for j in range(0, i):
				self.objects[i].collision_check(self.objects[j])
		# Finally, see whether any of the balls fall into goals
		i = 0
		while i < len(self.objects):
			if (not isinstance(self.objects[i], Ball)):
				i += 1
			else:
				# Does it fall into any of the goals?
				if self.objects[i].center.y > self.cy - 70 and self.objects[i].center.y < self.cy + 70:
					if self.objects[i].center.x < 5 + self.objects[i].radius:
						# Left goal:
						del self.objects[i]
						self.scoreLeft += 1
					elif self.objects[i].center.x > self.width - 5 - self.objects[i].radius:
						# Right goal
						del self.objects[i]
						self.scoreRight += 1
					else:
						i+=1
				else:
					i+=1
			

class WorldObject:
	"""
	This is a sample "root class" that can be used as an item in the world,
	(i.e. you can use world.add_object(o) for instances complying with this interface).
	You don't have to inherit from this class, but you have to make sure the conditions listed here
	are satisfied.
	"""
	def __init__(self, center, radius):
		"Every world object must have a center and radius for fallback collision detection. Note that coordinates are relative to the world."
		self.center = center
		self.radius = radius
	def draw(self, screen):
		"Draws the object on screen"
		pass
	def simulate(self):
		"Performs one step of object physics simulation. This is ALWAYS called before wall_check and collision_check"
		pass
	def wall_check(self, w):
		"""
		Given a wall object, checks for a collision and updates state, if necessary.
		Always called after simulate (i.e. you may precompute something there).
		"""
		pass
	def collision_check(self, o):
		"""
		Given any non-wall object, checks for a collision and updates state of this and colliding object, if necessary.
		Always called after simulate (i.e. may precompute something there).
		"""
		pass
		
# -------------- "Ball" and "Robot" are objects in the world -------------------
# Both know how to draw themselves, how to simulate themselves and how to react to
# collisions with walls and other objects
# Robot is given in <your_robot_name>.py

class Ball(WorldObject):
	"""The ball is the most basic world object"""
	def __init__(self, center, radius = 4.3):	# Actual radius is 43/2 mm, i.e. 4.3 pixels
		WorldObject.__init__(self, center, radius)
		self.v = Point(0, 0)	# Speed
	def draw(self, screen):
		ORANGE = (255,60,0)
		draw.circle(screen, ORANGE, self.center.as_tuple(), int(self.radius))
	def simulate(self):
		"Ball's movement is simple linear integration with coulomb friction"
		# We have measured that a typical ball has a friction deceleration of about 
		# -0.25 m/s^2. This is equal to 250 mm / 1mln ms^2 = 50pixels / 1000000 ms^2 = 0.00005 px/ms^2
		FRICTION_FORCE = 0.00005
		n = self.v.norm()
		if (n > 0):
			# Move
			self.center.add(self.v)
			# Account for friction
			new_n = n - FRICTION_FORCE # Coulomb friction reduces the length of the vector uniformly each step
			if (new_n < 0):
				new_n = 0
			self.v.mul(new_n/n)
	def wall_check(self, w):
		"""
		W is a Wall object. We may safely assume that our location is always to the right of the wall (i.e. distance should be positive).
		"""
		# Compute distance to the wall
		d = w.dist_to_point(self.center)
		if (d < self.radius):
			# Nudge so that we are always to the right side of the ball
			self.center.add(w.normal*(self.radius - d))
			# If our velocity vector is pointing towards the wall, reverse it
			wall_v = self.v.inner_product(w.normal)
			if wall_v < 0:
				self.v.add(w.normal*(-2*wall_v))
	def collision_check(self, obj):
		# If object is not a Ball, then let him do the collision computation
		if (not isinstance(obj, Ball)):
			return obj.collision_check(self)	# NB: Only balls are allowed to do this trick, otherwise we'll get an infinite cycle here
		else:
			# It is a ball. First see whether we are nearby
			direction = obj.center - self.center # Arrow pointing from "us" to "them"
			dist = direction.norm()			
			if (dist < self.radius + obj.radius):
				# Yes, it's a collision. Resolve it. 
				# We assume all balls of equal weight, hence the collision resolution is fairly simple.
				# First we nudge the offending ball slightly to remove the collision
				direction_normalized = direction * (1/dist)
				obj.center.add(direction_normalized*(self.radius + obj.radius - dist))
				# Next let us look at that guy's speed from our perspective
				his_v = obj.v - self.v
				# Is it moving towards us?
				towards_v = his_v.inner_product(direction_normalized)
				if (towards_v < 0):
					# Yep, we must fix this. What we do is we steal this component of his velocity
					steal_v = direction_normalized*towards_v
					obj.v.add(steal_v*(-1))
					self.v.add(steal_v)

if __name__ == "__main__":
	# Run doctests (hint, run with -v for verbose output)
	import doctest
	doctest.testmod()
