# Tegemist on Team Spiriti modifitseeritud roboti failiga
# Selles failis ei sisaldu classi RobotServer()

import 	pygame, random, thread, traceback
from 	pygame.locals 	import *
from 	pygame 			import draw
from 	math 			import sin, cos, sqrt
from 	world 			import Point, Wall, WorldObject, Ball

class Robot(WorldObject):
	# Robot must fit into a 350mm cylinder, which here means that it should not exceed a square of 49x49 pixels more or less. Hence the width/height parameters.
	def __init__(self, world, name, role):
		width  = 40 
		height = 40
		center 					= Point(int(12+height/2), int(12+width/2)) if role == "TOPLEFT" else Point(world.width-(12+height/2), world.height-(12+width/2))
		
		WorldObject.__init__(self, center, int(sqrt(width**2 + height**2)/2) )
		self.world = world
		self.name = name
		self.wr = width/2
		self.hr = height/2
		self.beacon_point 		= Point(world.width + 50, world.cy) if role == "TOPLEFT" else Point(-50, world.cy)
		self.beacon_point_en 	= Point(0, world.cy) 				if role == "TOPLEFT" else Point(world.width, world.cy)
		self.goal_center 		= Point(world.width, world.cy) 		if role == "TOPLEFT" else Point(0, world.cy)		
		
		
		# "Forward" is the unit direction vector where the robot is facing
		# (0, 1) is the "default" (height along the Y axis)
		self.forward = Point(0, 1)
		self.left = Point(-self.forward.y, self.forward.x)
		if role == "TOPLEFT":
			self.rotate(3.1415/2)
		else:
			self.rotate(-3.1415/2)

		# Those are state parameters
		self.leftSpeed = 0
		self.rightSpeed = 0
		
		# Camera sensor parameters
		self.CAMERA_DEPTH 	= 800 	# Kaugus kuhu naeme 
		self.CAMERA_SIDE 	= 500	# Laius kuhu naeme
		
		# Write concurrency lock (we assume reads are atomic, and even if not, just ignore read errors)
		self.data_lock = thread.allocate_lock()

		# Whether there's a ball in the grabber
		self.grabbed_ball = None
		self.grabbed_ball_lock = thread.allocate_lock()
		
	def draw(self, screen):
		# Center point
		draw.line(screen, (0,0,0), (self.center - self.left*5).as_tuple(), (self.center + self.left*5).as_tuple())
		draw.line(screen, (0,0,0), self.center.as_tuple(), (self.center + self.forward*8).as_tuple())
		# Sensor edges
		draw.line(screen, (255,0,0), self.center.as_tuple(), (self.center + self.forward*self.CAMERA_DEPTH + self.left*self.CAMERA_SIDE).as_tuple())		
		draw.line(screen, (255,0,0), self.center.as_tuple(), (self.center + self.forward*self.CAMERA_DEPTH + self.left*(-self.CAMERA_SIDE)).as_tuple())	
		draw.line(screen, (255,0,0), (self.center + self.forward*self.CAMERA_DEPTH + self.left*(-self.CAMERA_SIDE)).as_tuple(), \
									(self.center + self.forward*self.CAMERA_DEPTH + self.left*self.CAMERA_SIDE).as_tuple())
		# Beacon line
		if (self.beacon()):
			draw.line(screen, (255, 100, 0), self.center.as_tuple(), self.beacon_point.as_tuple())
		# Wheels
		for side in [1, -1]:
			draw.line(screen, (0,0,0), (self.center + self.left*self.wr*side - self.forward*8).as_tuple(), (self.center + self.left*self.wr*side + self.forward*8).as_tuple(), 5)
		# Sides
		for (f,t) in self.edges():
			draw.line(screen, (0,0,0), f.as_tuple(), t.as_tuple())

	def edges(self):
		"""Enumerate edges as tuples ((x,y), (x,y)) in clockwise order [assuming mathematical coordinates]"""
		leftback = self.center + self.left*self.wr - self.forward*self.hr
		leftfront = self.center + self.left*self.wr + self.forward*self.hr
		rightfront = self.center - self.left*self.wr + self.forward*self.hr
		rightback = self.center - self.left*self.wr - self.forward*self.hr
		yield (leftback, leftfront)
		yield (leftfront, rightfront)
		yield (rightfront, rightback)
		yield (rightback, leftback)
		
	def rotate(self, angle):
		self.forward.rotate(angle)
		self.left.rotate(angle)
		
	def simulate(self):
		# This is a hack which only works at small simulation steps
		leftTurn = (self.leftSpeed - self.rightSpeed)/self.wr/2
		forwardMove = (self.leftSpeed + self.rightSpeed)/2
		self.v = self.forward*forwardMove
		if (self.v != 0):
			self.center.add(self.v)
		if (leftTurn != 0):
			self.forward = self.forward + self.left*leftTurn
			self.forward.normalize()
			self.left = Point(-self.forward.y, self.forward.x)
			
		# If there is a grabbed ball, carry it around
		with self.grabbed_ball_lock:
			if self.grabbed_ball is not None:
				self.grabbed_ball.v = Point(0,0)
				self.grabbed_ball.center = self.center + self.forward*self.grabbed_forward + self.left*self.grabbed_left
	
		# Precompute "edge walls", those will be useful in collision checks
		self.edge_walls = [Wall(e[1], e[0]) for e in self.edges()]
		all_x = [w.p1.x for w in self.edge_walls]
		all_y = [w.p1.y for w in self.edge_walls]
	
	def wall_check(self, w):
		# Robot's wall handling is fairly trivial. If we see we're hitting the wall, we'll nudge back from it
		m = min([w.dist_to_point(edge[0]) for edge in self.edges()])
		if (m < 0):
			# We need to nudge perpendicular to the wall by distance -m
			self.center.add(w.normal*(-m))
	
	def collision_check(self, obj):
		# If it is not a ball, ignore it
		if not isinstance(obj, Ball): # objekt ei ole pall
			# Just check the "bounding circle"
			dir = obj.center - self.center
			dist = dir.norm()
			if (dist < obj.radius + self.radius):
				# Nudge either us or them, choose randomly to avoid some ugliness
				dir_normalized = dir * (1/dist)
				if (random.randint(0,1) == 0):
					# Them
					obj.center.add(dir_normalized*(obj.radius + self.radius - dist))
				else:
					# Us
					self.center.add(dir_normalized*(dist - (obj.radius + self.radius)))
		else: #objekt on pall
			# Find which wall is the ball touching
			for w in self.edge_walls:
				d = w.dist_to_point(obj.center) - obj.radius
				if (d > -0.3 and d < 0):
					# Is the ball within the range of the wall at all?
					wall_coord = (obj.center - w.p1).inner_product(w.v_normalized)
					if (wall_coord >= -obj.radius and wall_coord <= w.len+obj.radius):
						# Yes, it does, something must be done.
						# Nyyd kontrollime kas esimene ots?
						v = obj.center - self.center
						v_left = self.left.inner_product(v)
						v_forward = self.forward.inner_product(v)
						if (abs(v_left) < self.wr - 2):
							# OK, grab
							with self.grabbed_ball_lock:
								self.grabbed_ball = obj
								obj.v = Point(0, 0)
								self.grabbed_forward = v_forward - 5
								self.grabbed_left = v_left
							return
						else: #kui pole esimene ots	
							# First, nudge
							obj.center.add(w.normal * (-d))
							# Second, simulate a rebounce (this is a hack, but it's way easier than considering rotations and stuff)
							obj.v.add(w.normal * (-d*2))

	# ----------- The following are the main commands for the robot -----------------
	
	def wheels(self, left, right):
		"""
			Sets the wheel speed. Left and right must be numbers -100..100.
			100 is 1 m/s.
		"""
		with self.data_lock:
			self.leftSpeed 	= left	/500.0	# leftSpeed and rightSpeed are in pixels per second
			self.rightSpeed = right	/500.0
	def grab(self):
		"If at the moment this function is called there is a ball right at the front of the robot, the ball is 'grabbed'"
		if self.grabbed_ball is not None: #pall on juba haaratud, meil peaks olema, et votab koik pallid
			return
		for b in self.world.objects:
			if isinstance(b, Ball):
				# First check distance to center
				v = b.center - self.center
				v_forward = self.forward.inner_product(v)
				if v_forward > 0 and v_forward < self.hr + b.radius + 3:
					# See whether the ball is within the front edge
					v_left = self.left.inner_product(v)
					if (abs(v_left) < self.wr - 2):
						# OK, grab
						with self.grabbed_ball_lock:
							self.grabbed_ball = b
							b.v = Point(0, 0)
							self.grabbed_forward = v_forward - 5
							self.grabbed_left = v_left
						return
	def beacon(self):
		"Returns true if cos(angle) to beacon is > 0.99"
		dbeacon = self.beacon_point - self.center
		dbeacon.normalize()
		#originaalis oli 0.99 mis vastab 8.1 kraadile meie kasutame 0.995 mis vastab 5.7le, 993-6.7kraadi
		return dbeacon.inner_product(self.forward) > 0.994
	def shoot(self):
		"If there is a grabbed ball, the ball is shot off"
		with self.grabbed_ball_lock:
			if self.grabbed_ball is not None:
				self.grabbed_ball.center = self.center + self.forward*(self.grabbed_forward + 10) + self.left*self.grabbed_left
				# Shoots the ball at 0.4 pixels per millisecond (2.0 m/s)
				self.grabbed_ball.v = self.forward * 1	## oli 0.4 mis vastab 2 ms, paneme 2 mis vastab 10m/s 
				self.grabbed_ball = None
	def camera(self):
		"This is the 'camera' sensor. If any ball is found in the 'camera triangle', the distance and bearing to it are reported (with a random 10% noise)"
		"If several balls are found, one of them is reported (typically it is a stable solution)"
		v_forward_closest 	= 5000;
		v_left_closest 		= 5000;
		for b in self.world.objects:
				if isinstance(b, Ball):
						# First check distance to center
						v = b.center - self.center
						v_forward = self.forward.inner_product(v)
						if v_forward > 0 and v_forward < self.CAMERA_DEPTH:
								# See whether the ball is within the triangle
								v_left = self.left.inner_product(v)
								tan = abs(v_left)/v_forward
								if (tan < float(self.CAMERA_SIDE)/self.CAMERA_DEPTH):
										# The ball is inside
										if (v_forward < v_forward_closest):
												v_forward_closest 	= v_forward
												v_left_closest 		= v_left
		if (v_forward_closest != 5000):
			return (v_forward_closest*random.uniform(0.9,1.1), v_left_closest*random.uniform(0.9,1.1))
		return None

	def optokatkesti(self):
		"See on optokatkesti kontrollimine. Kui pall on triblajas, siis peab vastama 1, kui ei ole siis False"
		if self.grabbed_ball is not None: #Pall on haaratud
			return 1

class RobotServer:
	"""
	This is the robot's network controller interface. It accepts commands over TCP and forwards them to the robot.
	Usage:
	  r = Robot( ... ) # create the robot instance
	  s = RobotServer(r, port=5000) # create the robot server
	  s.serve()		   # starts a new thread with the server. The thread runs forever.
	"""
	def __init__(self, robot, port=5000):
		self.robot = robot
		self.port = port
	def serve(self):
		"""Starts the server in a separate thread"""
		import thread
		thread.start_new_thread(self._server_thread, tuple())
	def _server_thread(self):
		# Echo server program
		print "Starting server at port %d" % self.port
		import socket
		
		HOST = ''       # Symbolic name meaning all available interfaces
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)	#Vabastamine pordi
		s.bind((HOST, self.port))
		s.listen(1)
		print "Robot %s listening at port %d" % (self.robot.name, self.port)
		while 1:
			try:
				conn, addr = s.accept()
				print 'Connected by', addr
				while 1:
					data = conn.recv(1024)
					if not data: break
					#print "%s<< %s" % (self.robot.name, data.strip())
					response = self._process_command(data)
					#print "%s>> %s" % (self.robot.name, response.strip())
					conn.send(response + "\n")
				conn.close()
			except:
				traceback.print_exc()
	
	def _process_command(self, cmd):
		"Reaction to each command"
		try:
			c = cmd.split()
			if c[0] == "WHEELS":
				l,r = int(c[1]), int(c[2])
				if (l < -100 or l > 100 or r < -100 or r > 100):
					return "ERROR"
				# Introduce up to 10% error in settings
				ltrue = l*random.uniform(0.9, 1.1)
				rtrue = r*random.uniform(0.9, 1.1)
				self.robot.wheels(ltrue, rtrue)
				return "OK"
			elif c[0] == "CAM":
				c = self.robot.camera()
				if c is None:
					return "0 0"
				else:
					return "%f %f" % c
			elif c[0] == "GRAB":
				self.robot.grab()
				return "OK"
			elif c[0] == "SHOOT":
				self.robot.shoot()
				return "OK"
			elif c[0] == "BEACON":
				return "1" if self.robot.beacon() else "0"
			elif c[0] == "OPTO":			#Lisatud optokatkesti
				return "1" if self.robot.optokatkesti() else "0"			
			else:
				return "ERROR: else"
		except:
			return "ERROR: yldine"			

