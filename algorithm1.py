import socket, sys
import time, random

class Algorithm:
	def __init__(self, port):
		self.port = port
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.connect(('localhost', port))

	def command(self, cmd):
		self.socket.send(cmd)
		result = self.socket.recv(1024)
		return result

class State:
	def __init__(self, a):
		self.a = a
	def command(self, cmd):
		return self.a.command(cmd)
	def next(self, state):
		self.a.state = state

class StateStop(State):
	# The stopped state
	def step(self):
		self.command("WHEELS 0 0")

class StateApproaching(State):
	# Assuming the ball is right in front, approaches it until the distance goes down to 30
	# If left/right position changes to exceed 5, shifts to Rotating.
	# If ball lost, shifts to Searching
	# When approach complete, shifts to stop
	def step(self):
		v = self.command("CAM")
		if v == "0 0\n":
			# Lost the ball!
			print "Lost the ball, going to search again"
			self.next(StateSearching(self.a))
		else:
			dist, left = map(float, v.split())
			print "Distance: %f" % dist
			if abs(left) > 5:
				print "Lost rotation, going to rotate again"
				self.next(StateRotating(self.a))
			if dist < 30:
				print "Approach complete, grabbing and turning until we see the beacon"
				time.sleep(0.1) # Wait just a bit before we grab
				self.command("GRAB")
				self.command("WHEELS 10 -10") # Start turning
				bcn = self.command("BEACON")
				if (int(bcn) == 1):
					print "Found beacon, shoot!"
					self.command("SHOOT")
					self.command("WHEELS 0 0")
					time.sleep(0.5)
					print "Now go looking for another ball"
					self.next(StateSearching(self.a))
			else:
				print "Continuing approach"
				self.command("WHEELS 40 40")
	
class StateRotating(State):
	# Assumes the ball is in the cam. Rotates to get it to the middle.
	# Once achieved, shifts to "approach". If ball lost, shifts to "searching"
	# Note, this may not be a good idea in practice, as it is possible that while rotating another
	# ball will be detected and the original ball is lost
	
	def step(self):
		v = self.command("CAM")
		if v == "0 0\n":
			# Lost the ball!
			print "Lost the ball, going to search again"
			self.next(StateSearching(self.a))
		else:
			left = float(v.split()[1])
			if abs(left) < 3:	# OK, rotated!
				print "Rotation complete, approaching..."
				self.next(StateApproaching(self.a))
			else:
				# Set wheel speed according to the side
				sgn = 1 if left > 0 else -1
				self.command("WHEELS %d %d" % (sgn*20, -sgn*20))

class StateSearching(State):
	# The searching state - robot will hectically wander along the field until it finds a ball in the cam
	# Once ball found, shifts to "rotating" state
	def __init__(self, a):
		self.a = a
		self.last_change = 0
		
	def step(self):
		# Check the cam
		v = self.command("CAM")
		# Do we see anything?
		if v != "0 0\n":
			# Yay, now rotate in position
			print "Ball found, now rotate in position.."
			self.next(StateRotating(self.a))
		else:
			# Keep wandering...
			curtime = time.time()
			if (curtime - self.last_change > 2):
				# Change direction
				self.command("WHEELS %d %d\n" % (random.randint(-10,100), random.randint(-10,100)))				
				self.last_change = curtime
		time.sleep(0.001)
	
class Algorithm1(Algorithm):
	def __init__(self, port):
		Algorithm.__init__(self, port)
		self.state = StateSearching(self)
	def run(self):
		print "Running algorithm"
		while 1:
			self.state.step()
	
def main():
	try:
		port = int(sys.argv[1])
	except:
		print "Usage: ./algorithm.py <port>"
		return
	a = Algorithm1(port)
	a.run()

if __name__ == "__main__":
	main()