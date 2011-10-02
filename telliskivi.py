import pygame,random,thread,traceback
from pygame.locals import *
from pygame import draw
from math import sin, cos, sqrt, atan2
import thread

from world import Point, Wall, WorldObject, Ball

class Robot(WorldObject):
	# Robot must fit into a 350mm cylinder, which here means that it should not exceed a square of 49x49 pixels more or less. Hence the width/height parameters.
	def __init__(self, world, name, role):
		RADIUS = 130/5  # The diameter of the telliskivi robot is 260mm. In pixels it makes a radius of 26
		self.WHEEL_RADIUS = 105/5
		self.FORWARD_EDGE_FRONT = 85/5
		self.FORWARD_EDGE_LEFT = 100/5 # 98.86mm
		CENTER = Point(12+RADIUS, 12+RADIUS) if (role == "TOPLEFT") else Point(world.width - 12 - RADIUS, world.height - 12 - RADIUS)
		WorldObject.__init__(self, CENTER, RADIUS)
		self.world = world
		self.name = name
		self.wr = RADIUS
		self.hr = RADIUS
		self.beacon_point = Point(world.width + 50, world.cy) if role == "TOPLEFT" else Point(-50, world.cy)
		self.beacon_point_en = Point(0, world.cy) if role == "TOPLEFT" else Point(world.width, world.cy)
		self.goal_center = Point(world.width, world.cy) if role == "TOPLEFT" else Point(0, world.cy)
		
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
		self.CAMERA_DEPTH = 1000  # Let's say the camera sees as far as the end of the field (~5 meters = 1000 px)
		self.CAMERA_SIDE = 340    # The camera's side angle is 20 degree, i.e. at 1000 pixels it stretches to 340px
		
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
		# Circle
		# Angle we're pointing to:
		fwdangle = atan2(-self.forward.y, self.forward.x)
		fromangle = fwdangle + 0.86
		toangle = fwdangle - 0.86
		if (toangle < 0): toangle += 2*3.1415
		if (fromangle > toangle):
			fromangle -= 2*3.1415
		draw.arc(screen, (0,0,0), Rect(self.center.x - self.radius, self.center.y - self.radius, 2*self.radius, 2*self.radius), fromangle, toangle, 1)
				
		# Wheels
		for side in [1, -1]:
			draw.line(screen, (0,0,0), (self.center + self.left*self.WHEEL_RADIUS*side - self.forward*(30/5)).as_tuple(), (self.center + self.left*self.WHEEL_RADIUS*side + self.forward*(30/5)).as_tuple(), 5)
		# Sides
		for (f,t) in self.edges():
			draw.line(screen, (0,0,0), f.as_tuple(), t.as_tuple())

	def edges(self):
		"""Enumerate edges as tuples ((x,y), (x,y)) in clockwise order [assuming mathematical coordinates]"""
		frontleft = self.center + self.forward*self.FORWARD_EDGE_FRONT + self.left*self.FORWARD_EDGE_LEFT
		frontright = self.center + self.forward*self.FORWARD_EDGE_FRONT - self.left*self.FORWARD_EDGE_LEFT
		yield (frontleft, frontright)
		
	def rotate(self, angle):
		self.forward.rotate(angle)
		self.left.rotate(angle)
		
	def simulate(self):
		# This is a hack which only works at small simulation steps
		leftTurn = (self.leftSpeed - self.rightSpeed)/self.WHEEL_RADIUS/2
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
		m = w.dist_to_point(self.center) - self.radius
		if (m < 0):
			# We need to nudge perpendicular to the wall by distance -m
			self.center.add(w.normal*(-m))
	
	def collision_check(self, obj):
		# If it is not a ball, ignore it
		if not isinstance(obj, Ball):
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
		else:
			# First see whether the ball is within the radius
			dir = obj.center - self.center
			d = dir.norm() - obj.radius - self.radius
			if (d >= 0):
				return
			# Otherwise, check whether the ball is within the front edge part
			front_coord = dir.inner_product(self.forward)
			if (front_coord < self.FORWARD_EDGE_FRONT):
				# No, it is a side collision, rebound the ball
				dir.normalize()
				obj.center.add(dir * (-d))
				obj.v.add(dir * (-d*2))
				return
			# The ball might be touching the front edge, check it
			d = front_coord - self.FORWARD_EDGE_FRONT - obj.radius
			if (d < 0):
				left_coord = dir.inner_product(self.left)
				if (abs(left_coord) < self.FORWARD_EDGE_LEFT):
					# Kick from the front edge
					obj.center.add(self.forward * (-d))
					# Second, simulate a rebounce (this is a hack, but it's way easier than considering rotations and stuff)
					obj.v.add(self.forward * (-d*2))

	# ----------- The following are the main commands for the robot -----------------
	
	def wheels(self, left, right):
		"""
			Sets the wheel speed. Left and right must be numbers -100..100.
			100 is 1 m/s.
		"""
		with self.data_lock:
			self.leftSpeed = left/500.0    # leftSpeed and rightSpeed are in pixels per millisecond  1m/s = 1000mm/s = 200px/s = 0.2px/ms
			self.rightSpeed = right/500.0  # Hence, 100 input units are mapped to --> 0.2 leftSpeed/rightSpeed units.
	
	def grab(self):
		"If at the moment this function is called there is a ball right at the front of the robot, the ball is 'grabbed'"
		if self.grabbed_ball is not None:
			return
		for b in self.world.objects:
			if isinstance(b, Ball):
				# First check distance to center
				v = b.center - self.center
				v_forward = self.forward.inner_product(v)
				if v_forward > 0 and v_forward < self.FORWARD_EDGE_FRONT + b.radius + 3:
					# See whether the ball is within the front edge
					v_left = self.left.inner_product(v)
					if (abs(v_left) < self.FORWARD_EDGE_LEFT - 2):
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
		dbeacon_forward = self.forward.inner_product(dbeacon)
		dbeacon_left = self.left.inner_product(dbeacon)
		dbeacon_en = self.beacon_point_en - self.center
		dbeacon_en_forward = self.forward.inner_product(dbeacon_en)
		dbeacon_en_left = self.left.inner_product(dbeacon_en)
		dbeacon.normalize()
		dbeacon_en.normalize() 
		if dbeacon.inner_product(self.forward) < 0.9:
				dbeacon_forward = 5000
				dbeacon_left = 500
		if dbeacon_en.inner_product(self.forward) < 0.9:
				dbeacon_en_forward = 5000
				dbeacon_en_left = 500
		return [dbeacon_forward, dbeacon_left, dbeacon_en_forward, dbeacon_en_left]

	def goal(self):
		"Returns the location of the goal, in robot coordinates, if it is visible. Otherwise returns 0 0"
		dgoal = self.goal_center - self.center
		goal_dist  = dgoal.inner_product(self.forward)
		goal_left  = dgoal.inner_product(self.left)
		if (goal_dist < 1):
			return (0, 0)
		elif (abs(goal_left/goal_dist) > float(self.CAMERA_SIDE)/self.CAMERA_DEPTH): # Is the goal within viewing limits?
			return (0, 0)
		else:
			return (goal_dist, goal_left)
		
	def shoot(self):
		"If there is a grabbed ball, the ball is shot off"
		with self.grabbed_ball_lock:
			if self.grabbed_ball is not None:
				self.grabbed_ball.center = self.center + self.forward*(self.grabbed_forward + 10) + self.left*self.grabbed_left
				# Shoots the ball at 0.4 pixels per millisecond (2.0 m/s)
				self.grabbed_ball.v = self.forward * 0.4	
				self.grabbed_ball = None
	
	def camera(self):
		"This is the 'camera' sensor. If any ball is found in the 'camera triangle', the distance and bearing to it are reported (with a random 10% noise)"
		"If several balls are found, the closest is reported"
		v_forward_closest = 5000;
		v_left_closest = 5000;
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
												v_forward_closest = v_forward
												v_left_closest = v_left
		if (v_forward_closest != 5000):
			return (v_forward_closest*random.uniform(0.9,1.1), v_left_closest*random.uniform(0.9,1.1))
		return None

		
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
						return "-1 -1"
				else:
						return "%f %f" % c
			elif c[0] == "GRAB":
				self.robot.grab()
				return "OK"
			elif c[0] == "SHOOT":
				self.robot.shoot()
				return "OK"
			elif c[0] == "BEACON":
				#return "1" if self.robot.beacon() else "0"
				b = self.robot.beacon()
				return "%f %f %f %f" % (b[0], b[1], b[2], b[3])
			else:
				return "ERROR"
		except:
			return "ERROR"
