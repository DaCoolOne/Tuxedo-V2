import math
import os
import numpy as np

from rlbot.utils.structures.game_data_struct import GameTickPacket

from Structs import *

boost_accel = 991.66

# Got this function online
def float_range(start, stop, step):
	while start < stop:
		yield float(start)
		start += step

def pos(p):
	return p.physics.location

def vel(p):
	return p.physics.velocity

def sign(n):
	return 1 if n > 0 else -1

def constrain(n):
	return max(min(n, 1), -1)

def constrain_pi(n):
	while n > math.pi:
		n -= math.pi * 2
	while n < -math.pi:
		n += math.pi * 2
	return n

def correction(car, ideal):
	vec = ideal.align_from(car.physics.rotation)
	return constrain_pi(math.atan2(-vec.y, vec.x))

def correction2(rotation, ideal):
	vec = ideal.align_from(rotation)
	return constrain_pi(math.atan2(-vec.y, vec.x))

def clamp_abs(val, bound): # [-bound, bound] (inclusive)
	return min(max(-bound, val), bound)
def clamp(val, lower, upper): #[lower, upper] (inclusive), upper prioritized
	return min(max(lower, val), upper)

def correct(target, val, mult = 1):
	rad = constrain_pi(target - val)
	return (rad * mult)

# Projects into the future given a time, acceleration, and initial physics.
def project_future(packet, phys, t, a = None, gravity = True):
	if a is None:
		a = Vec3()
	if gravity:
		a.z = a.z + packet.game_info.world_gravity_z
	return Psuedo_Physics(
		velocity = a * t + phys.velocity,
		location = a * (t * t) + phys.velocity * t + phys.location,
		rotation = phys.rotation,
		angular_velocity = phys.angular_velocity
	)

# Thank you RLBot wiki! :D
def turn_radius(v):
	if v == 0:
		return 0
	return 1.0 / curvature(v)

# v is the magnitude of the velocity in the car's forward direction
def curvature(v):
	if 0.0 <= v < 500.0:
		return 0.006900 - 5.84e-6 * v
	elif 500.0 <= v < 1000.0:
		return 0.005610 - 3.26e-6 * v
	elif 1000.0 <= v < 1500.0:
		return 0.004300 - 1.95e-6 * v
	elif 1500.0 <= v < 1750.0:
		return 0.003025 - 1.10e-6 * v
	elif 1750.0 <= v < 2500.0:
		return 0.001800 - 0.40e-6 * v
	else:
		return 0.0

# Turning stuff
class Vec2:
	
	def __init__(self, x=0, y=0):
		self.x = x
		self.y = y
	
	def __add__(v1, v2):
		return Vec2(v1.x + v2.x, v1.y + v2.y)
	
	def __sub__(v1, v2):
		return Vec2(v1.x - v2.x, v1.y - v2.y)
	
	def __mul__(self, n):
		return Vec2(self.x * n, self.y * n)
	
	def length(self):
		return math.sqrt(self.x*self.x+self.y*self.y)
	
	def normal(self, n=1):
		length = n / self.length()
		return Vec2(self.x * length, self.y * length)
	
	def inflate(self, n=0):
		return Vec3(self.x, self.y, n)
	
	def cross(v1, v2):
		return v1.x * v2.y - v1.y * v2.x
	
	def cast(v):
		return Vec2(v.x, v.y)
	
	def rot90(v):
		return Vec2(v.y, -v.x)
	
	def dot(v1, v2):
		return v1.x * v2.x + v1.y * v2.y
	
	def angle_between(v1, v2):
		return math.acos(v1.normal().dot(v2.normal()))
	
	def angle(self):
		return math.atan2(self.y, self.x)
	

class Ray2D:
	
	def __init__(self, location = Vec2(), direction = Vec2()):
		self.location = Vec2.cast(location)
		self.direction = Vec2.cast(direction)
	
	# t = (q − p) × s / (r × s)
	def intersection(v1, v2):
		# Credits to stack overflow
		p = v1.location
		q = v2.location
		r = v1.direction
		s = v2.direction
		
		div_t = r.cross(s)
		
		if div_t == 0:
			return
		
		t = (q - p).cross(s) / div_t
		
		return p + (r * t)
		
	

class ArcTurn:
	
	def __init__(self, location, direction, point):
		
		# If something goes wrong, this will be false
		self.valid = False
		
		forward = Vec2.cast(direction)
		self.car = Ray2D(Vec2.cast(location), forward.rot90())
		self.target_loc = Vec2.cast(point)
		
		center = (self.car.location + self.target_loc) * 0.5
		c_dir = (self.car.location - self.target_loc).rot90()
		c_ray = Ray2D(center, c_dir)
		
		c = self.car.intersection(c_ray)
		
		if c is None:
			return
		
		self.center = c
		self.radius = (self.car.location - self.center).length()
		
		self.v1 = self.car.location - self.center
		self.v2 = self.target_loc - self.center
		
		ang = self.v1.angle_between(self.v2)
		
		if forward.dot(self.v2) > 0:
			self.arc_angle = abs(ang)
		else:
			self.arc_angle = abs(math.pi * 2 - ang)
		
		self.arc_length = self.arc_angle * self.radius
		
		self.valid = True
	
	def time_at_speed(self, s):
		return self.arc_length / s
	
	def render(self, renderer, color, total_points = 10):
		points = []
		
		if total_points > 2:
			start_angle = self.v1.angle()
			add = 1 if abs(constrain_pi(start_angle + self.arc_angle - self.v2.angle())) < math.pi * 0.5 else -1
			point_space = self.arc_angle / total_points
			for i in range(total_points):
				angle = start_angle + i * point_space * add
				points.append(UI_Vec3(self.center.x + math.cos(angle) * self.radius, self.center.y + math.sin(angle) * self.radius, 20))
		
		renderer.draw_polyline_3d(points, color)
	

# Rough approximator for if a point is in bounds. Used by Line_Arc_Line class
def point_in_bounds(point):
	return (abs(point.x) < 3996 and abs(point.y) < 5020) or (abs(point.x) < 750 and abs(point.y) < 5220)

class Line_Arc_Line:
	# offset represents line 1
	def __init__(self, car, target, offset):
		# Used to signal early aborts
		self.valid = True
		
		self.car = car
		
		self.start = Vec2.cast(car.physics.location)
		self.target = Vec2.cast(target)
		self.offset = Vec2.cast(offset)
		self.p2 = self.target + self.offset
		start_to_p2 = self.p2 - self.start
		start_v = car.physics.velocity.flatten().length()
		self.vel = Time_to_Pos(start_to_p2.length(), start_v, car.boost)
		
		v_upper = self.vel.velocity
		
		if v_upper <= 0:
			self.valid = False
			return
		
		self.arc_radius = turn_radius(v_upper) * 1.05 # Add a slight multiplier so that the cars can properly navigate the arc
		
		cp = self.offset.normal(self.arc_radius).rot90()
		
		if cp.dot(start_to_p2) > 0:
			cp *= -1
		
		self.arc_center = self.p2 + cp
		
		start_to_c = self.arc_center - self.start
		
		# Add sizable epsilon
		if start_to_c.length() <= self.arc_radius + 0.00001:
			self.valid = False
			return
		
		self.a2 = cp * -1
		
		offset_angle = math.acos(self.arc_radius / start_to_c.length())
		inner_angle = math.pi - offset_angle
		
		ang = (start_to_c * -1).angle()
		
		a1 = Vec2(math.cos(ang + inner_angle), math.sin(ang + inner_angle)) * self.arc_radius
		a2 = Vec2(math.cos(ang - inner_angle), math.sin(ang - inner_angle)) * self.arc_radius
		
		a1_dp = a1.dot(start_to_c)
		a2_dp = a2.dot(start_to_c)
		
		p1_a = a1 * (1 if a1_dp < 0 else -1)
		p1_b = a2 * (1 if a2_dp < 0 else -1)
		
		start_to_p1_a = (self.arc_center + p1_a - self.start)
		# start_to_p1_b = (self.arc_center + p1_b - self.start)
		
		# Arc direction
		self.arc_dir = sign(cp.dot(self.offset.rot90()) * -1)
		
		if sign(p1_a.rot90().dot(start_to_p1_a)) == self.arc_dir:
			self.a1 = p1_a
			self.p1 = self.arc_center + p1_a
		else:
			self.a1 = p1_b
			self.p1 = self.arc_center + p1_b
		
		# self.arc_length = constrain_pi(self.arc_center)
		
		angle_arc = self.a1.angle_between(self.a2)
		
		if self.offset.dot(self.p1 - self.arc_center) > 0:
			self.arc_length = self.arc_radius * angle_arc
		else:
			self.arc_length = self.arc_radius * (math.pi * 2 - angle_arc)
		
	
	def render(self, agent):
		render_star(agent, self.start.inflate(20), agent.renderer.green(), 30)
		render_star(agent, self.target.inflate(20), agent.renderer.green(), 30)
		render_star(agent, self.p1.inflate(20), agent.renderer.green(), 30)
		render_star(agent, self.p2.inflate(20), agent.renderer.green(), 30)
		
		# render_star(agent, self.arc_center.inflate(20), agent.renderer.red(), 30)
		
		agent.renderer.draw_string_3d(self.arc_center.inflate(20).UI_Vec3(), 2, 2, str(math.floor(math.degrees(self.arc_length / self.arc_radius))), agent.renderer.yellow())
		
		agent.renderer.draw_line_3d(self.arc_center.inflate(20).UI_Vec3(), self.p2.inflate(20).UI_Vec3(), agent.renderer.red())
		agent.renderer.draw_line_3d(self.arc_center.inflate(20).UI_Vec3(), self.p1.inflate(20).UI_Vec3(), agent.renderer.red())
		
		agent.renderer.draw_line_3d(self.target.inflate(20).UI_Vec3(), self.p2.inflate(20).UI_Vec3(), agent.renderer.blue())
		agent.renderer.draw_line_3d(self.start.inflate(20).UI_Vec3(), self.p1.inflate(20).UI_Vec3(), agent.renderer.blue())
		
	
	def calc_time(self, car):
		total_length = self.arc_length + (self.start + self.p1).length() # + self.offset.length()
		max_v = self.vel.velocity
		t1 = Time_to_Vel(max_v, car.physics.velocity.flatten().length(), car.boost)
		t2 = Time_to_Pos(total_length, car.physics.velocity.flatten().length(), car.boost)
		
		if t2.time < t1.time:
			return t2.time + (self.offset.length() / t2.velocity)
		
		length_to_max_speed = t1.location
		length_at_max_speed = total_length - length_to_max_speed
		
		return t1.time + (self.offset.length() + length_at_max_speed) / max_v
		
	
	def check_in_bounds(self):
		return point_in_bounds(
			self.target
		) and point_in_bounds(
			self.p1
		) and point_in_bounds(
			self.p2
		) and point_in_bounds(
			self.arc_center + Vec2( 1, 0) * self.arc_radius
		) and point_in_bounds(
			self.arc_center + Vec2(0,  1) * self.arc_radius
		) and point_in_bounds(
			self.arc_center + Vec2(-1, 0) * self.arc_radius
		) and point_in_bounds(
			self.arc_center + Vec2(0, -1) * self.arc_radius
		)
		
	

class Hit():
	def __init__(self, time = None, velocity = None, location = None):
		self.time = time
		self.velocity = velocity
		self.location = location

class AccelModelPoint():
	def __init__(self, line):
		nums = line.split(",")
		self.time = float(nums[0])
		self.velocity = float(nums[1])
		self.position = float(nums[2])

# For these utilities, I'm going to use a polyline approximation because the curves are rather complicated.
# First, though, we need to collect the acceleration graphs.
cwd = os.getcwd()
path = os.path.dirname(os.path.realpath(__file__))
os.chdir(path)

accel_f = open("acceleration.txt", "r")
boost_accel_f = open("boost_acceleration.txt", "r")

max_car_vel = 0

acceleration = []
for line in accel_f.readlines():
	if len(line) > 0:
		a = AccelModelPoint(line)
		acceleration.append(a)
		max_car_vel = max(max_car_vel, a.velocity)

# print(max_car_vel)

boost_acceleration = []
for line in boost_accel_f.readlines():
	if len(line) > 0:
		boost_acceleration.append(AccelModelPoint(line))

os.chdir(cwd)

def bin_search_time(arr, val, s=0, e=-1):
	if e < 0: e = len(arr)
	l = e - s
	div = s + math.floor(l * 0.5);
	if l == 1:
		return arr[div]
	elif arr[div].time > val:
		return bin_search_time(arr, val, s, div)
	else:
		return bin_search_time(arr, val, div, e)

def bin_search_velocity(arr, val, s=0, e=-1):
	if e < 0: e = len(arr)
	l = e - s
	div = s + math.floor(l * 0.5);
	if l == 1:
		return arr[div]
	elif arr[div].velocity > val:
		return bin_search_velocity(arr, val, s, div)
	else:
		return bin_search_velocity(arr, val, div, e)

def bin_search_position(arr, val, s=0, e=-1):
	if e < 0: e = len(arr)
	l = e - s
	div = s + math.floor(l * 0.5);
	if l == 1:
		return arr[div]
	elif arr[div].position > val:
		return bin_search_position(arr, val, s, div)
	else:
		return bin_search_position(arr, val, div, e)

# Arguments are passed in as scalars for convienience sake.
# Returns the ammount of time needed to drive a distance (length) giving a starting velocity (v) assuming no boost used.
# Uses the above velocity and position tables.
def Time_To_Pos_No_Boost(length, v):
	# Get start "time"
	start = bin_search_velocity(acceleration, v)
	
	# Calculate the end time
	end = bin_search_position(acceleration, start.position + length)
	
	# Add on some additional time for additional length traveled at max speed.
	extra = (start.position + length - end.position) / max(0.01, end.velocity)
	
	# Wow, that was easy :P
	return Hit(end.time - start.time + extra, end.velocity)

boost_consumption = 1/33
# Returns the ammount of time needed for a car to travel a length given a boost ammount and an initial velocity
def Time_to_Pos(length, v, boost):
	t_until_no_boost = boost * boost_consumption
	
	# First, assume unlimited boost
	
	# Get start "time"
	start = bin_search_velocity(boost_acceleration, v)
	
	# Calculate the end time
	end = bin_search_position(boost_acceleration, start.position + length)
	
	# Okay, now let's look at whether we run out of boost. :P
	if end.time - start.time > t_until_no_boost:
		# Oof, okay, let's evaluate our velocity when we run out of boost
		no_boost = bin_search_time(boost_acceleration, start.time + t_until_no_boost)
		if no_boost.velocity >= max_car_vel:
			# While technically there is a slight deceleration force applied to the car, for simplicity's sake we will simply assume that the car maintains its current speed
			extra = (start.position + length - no_boost.position) / max(0.01, no_boost.velocity)
			
			# Easy
			return Hit(end.time - start.time + extra, end.velocity)
		else:
			# Okay, so the car will now be accelerating for a bit. Fortunately, we already have a function for this, so we'll just...
			hit = Time_To_Pos_No_Boost(start.position + length - no_boost.position, no_boost.velocity)
			hit.time += t_until_no_boost
			return hit
		
	else:
		
		# Add on some additional time for additional length traveled at max speed.
		extra = max(0, start.position + length - end.position) / max(0.01, end.velocity)
		
		# Easy
		return Hit(end.time - start.time + extra, end.velocity)
	

def Time_To_Vel_No_Boost(vel, v):
	# Get start "time"
	start = bin_search_velocity(acceleration, v)
	
	# Calculate the end time
	end = bin_search_velocity(acceleration, vel)
	
	# Wow, that was easy :P
	return Hit(end.time - start.time, location = end.position - start.position)

boost_consumption = 1/33
# Returns the ammount of time needed for a car to travel a length given a boost ammount and an initial velocity
def Time_to_Vel(vel, v, boost):
	t_until_no_boost = boost * boost_consumption
	
	# First, assume unlimited boost
	
	# Get start "time"
	start = bin_search_velocity(boost_acceleration, v)
	
	# Calculate the end time
	end = bin_search_position(boost_acceleration, vel)
	
	# Okay, now let's look at whether we run out of boost. :P
	if end.time - start.time > t_until_no_boost:
		# Oof, okay, let's evaluate our velocity when we run out of boost
		no_boost = bin_search_time(boost_acceleration, start.time + t_until_no_boost)
		if no_boost.velocity > max_car_vel:
			# Easy
			return Hit(end.time - start.time, location = end.position - start.position)
		else:
			# Okay, so the car will now be accelerating for a bit. Fortunately, we already have a function for this, so we'll just...
			hit = Time_To_Vel_No_Boost(start.velocity + vel - no_boost.velocity, no_boost.velocity)
			hit.time += no_boost.time - start.time
			hit.location += no_boost.position - start.position
			return hit
		
	else:
		# Easy
		return Hit(end.time - start.time, location = end.position - start.position)
	

class Touch:
	def __init__(self, time = 0, location = Vec3(), is_garunteed = False, can_save = True):
		self.time = time
		self.location = location
		self.is_garunteed = is_garunteed
		self.can_save = can_save
	
	def to_numpy(self):
		n = np.zeros(6)
		n[0] = self.time
		n[1] = self.location.x
		n[2] = self.location.y
		n[3] = self.location.z
		n[4] = self.is_garunteed
		n[5] = self.can_save
		return n
	
	def from_numpy(n):
		return Touch(n[0], Vec3(n[1], n[2], n[3]), n[4], n[5])
	
	def copy(self):
		return Touch(self.time, self.location, self.is_garunteed, self.can_save)
	
	def recalculate_time(self, dt):
		self.time -= dt

# Determines time for car to hit a position on the ground.
def calc_hit(car, position, minimum = False, angle_correct = True):
	car_vel = car.physics.velocity
	car_vel_len = car_vel.length()
	car_pos = car.physics.location
	car_face = Vec3(1, 0, 0).align_to(car.physics.rotation)
	
	# Calculate the length of the car's path
	car_path = (position - car_pos).flatten()
	angle = car_vel.angle_between(car_path)
	
	# Length of the path the car must travel
	length = max(0, car_path.length() - (150 if minimum else 0)) # Allows us to fudge the numbers a bit
	
	if minimum and angle_correct:
		# Calculate that we don't need to turn the whole way to hit.
		if length > 200:
			turn2 = math.tan(150 / length)
			angle = max(0, angle - turn2)
		else:
			# So close that turning doesn't need to be taken into account
			angle = 0
	
	turn = turn_radius(car_vel_len) * angle
	
	# Calculate the time to get to the position
	turn_time = turn / max(1000, car_vel_len)
	drive_time = Time_to_Pos(max(0.01, length), car_vel_len, car.boost) if car_face.dot(car_vel) > 0.0 else Time_To_Pos_No_Boost(max(0.01, length), car_vel_len)
	
	drive_time.time += turn_time # + air_time
	
	return drive_time

def calc_path_length(car, position):
	
	car_pos = car.physics.location
	car_vel = car.physics.velocity
	car_path = (position - car_pos).flatten()
	angle = car_vel.angle_between(car_path)
	
	length = max(0, car_path.length() + 50)
	
	turn2 = math.tan(150 / length)
	angle = max(0, angle - turn2)
	
	turn = turn_radius(car_vel.length()) * angle
	
	return turn + length

class Simple_Hit_Prediction:
	def __init__(self, hit_prediction = None):
		if not hit_prediction is None:
			self.hit_time = hit_prediction.hit_time
			self.hit_game_seconds = hit_prediction.hit_game_seconds
			self.hit_velocity = hit_prediction.hit_velocity
			self.hit_position = hit_prediction.hit_position
			self.hit_index = hit_prediction.hit_index
		else:
			self.hit_time = 0
			self.hit_game_seconds = 0
			self.hit_velocity = 0
			self.hit_position = Vec3()
			self.hit_index = 0
	
	def recalculate_time(self, game_time):
		self.hit_time = self.hit_game_seconds - game_time
	
	def to_numpy(self):
		n = np.zeros(6)
		n[0] = self.hit_game_seconds
		n[1] = self.hit_velocity
		n[2] = self.hit_position.x
		n[3] = self.hit_position.y
		n[4] = self.hit_position.z
		n[5] = self.hit_index
		return n
	
	def from_numpy(n, game_time):
		s = Simple_Hit_Prediction()
		s.hit_time = n[0] - game_time
		s.hit_game_seconds = n[0]
		s.hit_velocity = n[1]
		s.hit_position = Vec3(n[2], n[3], n[4])
		s.hit_index = n[5]
		return s
	
	def ball_at_t(self, agent, packet, prediction, time, offset = None, max_height = 265):
		if offset is None:
			offset = Vec3()
		
		slice = Get_Ball_At_T(packet, prediction, min(time, self.hit_time))
		
		hit_car = packet.game_cars[self.hit_index]
		
		goal_pos = agent.field_info.my_goal.location
		goal_pos.z = min(self.hit_position.z, max_height)
		goal_pos.x = clamp_abs(self.hit_position.x, 700)
		
		if hit_car.has_dribble:
			goal_vec = (goal_pos - self.hit_position)
			goal_vec_2 = (goal_pos - self.hit_position + Vec3(1000, 0, 0))
			v = self.hit_position - hit_car.physics.location
			
			if goal_vec.angle_between(v) > goal_vec.angle_between(goal_vec_2):
				shot_vec = goal_vec.normal(self.hit_velocity * 1.5)
			else:
				shot_vec = v.normal(self.hit_velocity)
			
		else:
			goal_vec = (goal_pos - self.hit_position)
			shot_vec = goal_vec.normal(self.hit_velocity * 1.5)
		
		return (slice.physics.location if time < self.hit_time else slice.physics.location + shot_vec * (time - self.hit_time)) + offset

# Predictions from this will be a little off. Need to make it take into account the change of position in the turn
class Hit_Prediction(Simple_Hit_Prediction):
	def __init__(self, agent, packet):
		self.prediction = agent.ball_prediction
		team = agent.team
		current_time = packet.game_info.seconds_elapsed
		self.hit_time = 0
		for i in range(self.prediction.num_slices):
			slice = self.prediction.slices[i]
			if slice.game_seconds < current_time:
				continue
			
			for index, car in enumerate(packet.game_cars):
				if car.team == team or car.is_demolished:
					continue
				
				loc = slice.physics.location
				t = slice.game_seconds - current_time
				if loc.z > 265:
					air_hit = self.calc_air(packet, car, loc, t, packet.game_info.world_gravity_z)
					if (air_hit.velocity.length() <= 1000 and t < car.boost * (1/33)) and not car.has_wheel_contact:
						# Add a maximum velocity check
						if project_future(packet, car.physics, t, air_hit.velocity).velocity.length() < 2300:
							self.hit_time = slice.game_seconds - current_time
							self.hit_game_seconds = slice.game_seconds
							self.hit_position = loc
							self.hit_velocity = car.physics.velocity.length() + slice.physics.velocity.length() * 0.6 + 2000
							self.hit_car = car
							self.hit_index = index
				else:
					hit = calc_hit(car, loc, minimum = True)
					if hit.time < slice.game_seconds - current_time:
						# print(slice.game_seconds - current_time)
						self.hit_time = slice.game_seconds - current_time
						self.hit_game_seconds = slice.game_seconds
						self.hit_position = loc
						self.hit_velocity = min(2400, hit.velocity + slice.physics.velocity.length() * 0.6 + 1000) # Add 500 for flipping (Need to update to take into account direction of hit)
						self.hit_car = car
						self.hit_index = index
			
			if not self.hit_time and i >= self.prediction.num_slices - 1:
				self.hit_time = 6
				self.hit_position = self.prediction.slices[i].physics.location
				self.hit_game_seconds = current_time + 6
				self.hit_velocity = 0
				self.hit_car = packet.game_cars[0]
				self.hit_index = 0
			
			if self.hit_time:
				break
	
	def draw_path(self, renderer, c):
		poly = []
		for i in range(0, self.prediction.num_slices, 3):
			slice = self.prediction.slices[i]
			if slice.game_seconds > self.hit_game_seconds:
				break
			poly.append(slice.physics.location.UI_Vec3())
		renderer.draw_polyline_3d(poly, c)
		renderer.draw_line_3d(poly[len(poly) - 1], self.hit_position.UI_Vec3(), c)
	
	# Designed for aerials. Calculates the delta v to hit a location at a time.
	def calc_air(self, packet, car, position, time, grav_z, leniency = False):
		
		car_up = Vec3(0, 0, 1).align_to(car.physics.rotation)
		
		phys = Psuedo_Physics(
			location = car.physics.location,
			velocity = car.physics.velocity,
			rotation = car.physics.rotation,
			angular_velocity = car.physics.angular_velocity,
		)
		if car.has_wheel_contact:
			# More accurate simulation of the jump
			phys.velocity += car_up * 300
			phys = project_future(packet, phys, 0.2, car_up * 1400)
			time -= 0.2
		
		dv = delta_v(phys, position, max(0.0001, time), grav_z)
		
		if dv.length() > 1000 or (leniency and dv.length() >= 800):
			phys.velocity += car_up * 300
			dv = delta_v(phys, position, max(0.0001, time), grav_z)
		
		return Hit(velocity = dv)
	
	# Return the earliest touch time along with a flag for viability
	def get_earliest_touch(self, agent, packet, car, max_height = 4000, offset = None):
		if offset is None:
			offset = Vec3()
		current_time = packet.game_info.seconds_elapsed
		
		goal_pos = agent.field_info.my_goal.location
		goal_pos.z = min(self.hit_position.z, max_height)
		goal_pos.x = clamp_abs(self.hit_position.x, 700)
		
		if self.hit_car.has_dribble:
			goal_vec = (goal_pos - self.hit_position)
			# goal_vec_2 = (goal_pos - self.hit_position + Vec3(1000, 0, 0))
			# v = self.hit_position - self.hit_car.physics.location
			
			# if goal_vec.angle_between(v) > goal_vec.angle_between(goal_vec_2):
			shot_vec = goal_vec.normal(self.hit_car.physics.velocity.length())
			# else:
				# shot_vec = v.normal(self.hit_velocity)
			
		else:
			goal_vec = (goal_pos - self.hit_position)
			shot_vec = goal_vec.normal(self.hit_velocity * 1.5)
			
			
		
		for i in range(self.prediction.num_slices):
			slice = self.prediction.slices[i]
			if slice.game_seconds < current_time:
				continue
			loc = (slice.physics.location if slice.game_seconds < self.hit_game_seconds else self.hit_position + shot_vec * (slice.game_seconds - self.hit_game_seconds)) + offset
			t = slice.game_seconds - current_time
			if loc.z <= 265:
				#car_strike_loc = get_car_strike_loc(loc, packet, car)
				#hit = self.calc_hit(car, car_strike_loc)
				hit = calc_hit(car, loc, minimum = True)
				if (hit.time < slice.game_seconds - current_time and loc.z < max_height) or i >= self.prediction.num_slices - 3 or abs(loc.y) > 5120:
					return Touch(max(t, hit.time), loc, t <= self.hit_time, abs(loc.y) < 5120)
				
			elif slice.game_seconds - current_time > 1:
				air_hit = self.calc_air(packet, car, loc, slice.game_seconds - current_time, packet.game_info.world_gravity_z, leniency = True)
				if (air_hit.velocity.length() < 800 and t < car.boost * 0.033 and loc.z < max_height) or i >= self.prediction.num_slices - 3 or abs(loc.y) > 5120:
					if project_future(packet, car.physics, t, air_hit.velocity).velocity.length() < 2300:
						return Touch(t, loc, t <= self.hit_time, abs(loc.y) < 5120)
		
		return Touch(6, goal_pos, False, True)
	
	def get_simple(self):
		return Simple_Hit_Prediction(self)
	

def render_star(self, position: Vec3, color, size = 100):
	
	v = Vec3(1, 1, 1).normal(size)
	
	self.renderer.draw_line_3d((position - Vec3(size, 0, 0)).UI_Vec3(), (position + Vec3(size, 0, 0)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(0, size, 0)).UI_Vec3(), (position + Vec3(0, size, 0)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(0, 0, size)).UI_Vec3(), (position + Vec3(0, 0, size)).UI_Vec3(), color)
	
	self.renderer.draw_line_3d((position - Vec3(-v.x, v.y, v.z)).UI_Vec3(), (position + Vec3(-v.x, v.y, v.z)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(v.x, -v.y, v.z)).UI_Vec3(), (position + Vec3(v.x, -v.y, v.z)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(v.x, v.y, -v.z)).UI_Vec3(), (position + Vec3(v.x, v.y, -v.z)).UI_Vec3(), color)
	

def delta_v(phys, position, time, grav_z, car_vel = None):
	if not car_vel:
		car_vel = phys.velocity
	car_pos = phys.location
	return Vec3(
		(position.x - car_vel.x * time - car_pos.x) / (0.5 * time * time),
		(position.y - car_vel.y * time - car_pos.y) / (0.5 * time * time),
		(position.z - car_vel.z * time - car_pos.z) / (0.5 * time * time) - grav_z,
	)

def grounded_and_wall_check(agent,packet):
	my_car = packet.game_cars[agent.index]
	return my_car.has_wheel_contact, my_car.physics.location.z >= 50

def impulse_velocity(packet, phys, point, time):
	phys_pos = phys.location
	
	phys_to_ball = point - phys_pos
	
	impulse_2D = phys_to_ball.flatten()
	
	impulse_2D *= (1 / max(0.0001, time))
	
	# Worked this out a while ago
	z_vel = -(0.5 * packet.game_info.world_gravity_z * time * time - phys_to_ball.z) / max(0.0001, time)
	
	return Vec3(impulse_2D.x, impulse_2D.y, z_vel)

# Works decently to steer on ground
def steer_for_heading_err(headingErrRad):
	return constrain(-sign(headingErrRad) * (10.0 * (abs(headingErrRad) ** 1.2)))

def get_heading_err(current: Vec3, ideal: Vec3) -> float:
	# Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.
	
	# The in-game axes are left handed, so use -x
	current_in_radians = math.atan2(current.y, -current.x)
	ideal_in_radians = math.atan2(ideal.y, -ideal.x)
	
	diff = ideal_in_radians - current_in_radians
	
	# Make sure that diff is between -pi and +pi.
	if abs(diff) > math.pi:
		if diff < 0:
			diff += 2 * math.pi
		else:
			diff -= 2 * math.pi
	
	return diff

#Sign of y position of team net
def team_mult(team):
	if team == 1: # Orange
		return 1.0
	else: # 0==Blue
		return -1.0

def accel_at_spd(s): # Acceleration from throttle=1 at speed s (scales linearly with throttle)
	if s == 0:
		return 1600
	s = abs(s)
	if s < 1400: # 0 < |s| < 1400
		return 1592.0 - (s * (1432.0 / 1400.0)) # 1600 - (1440 * (|s| / 1400)); 1440 / 1400 = 36.0 / 35.0
	if s < 1410: # 1400 <= |s| < 1410
		return 22560.0 - (s * 16.0) # 160 * ((1410 - |s|) / 10)
	return 0 # |s| >= 1410

def line_drive_duration(v_i, d, boost_allotted): # Initial speed v_i, distance to travel d
	d_target = abs(d)
	dt = 1.0 / 120.0
	dt2 = 1.0 / (120.0 ** 2.0) # dt squared
	boost_usage_rate = 33.3
	thr = 1 # Throttle
	
	d_curr = 0						# Current distance
	v_curr = v_i					# Current velocity
	a_curr = thr * accel_at_spd(v_i)# Current acceleration
	boost_remaining = clamp(boost_allotted, 0, 100)
	frames = 0 # Current frame to simulate
	while d_curr < d_target:
		d_curr += (v_curr * dt) + 0.5 * (a_curr * dt2) # d_new = d + vt + 0.5at^2
		boosting = False
		if boost_remaining > 0: # Use boost
			boosting = True
			boost_remaining -= boost_usage_rate * dt
		a_curr = thr * accel_at_spd(v_curr) # Acceleration from throttle
		if boosting:
			a_curr += 991.667 # Boost acceleration
		
		v_curr_max = 2300.0 if boosting else max(v_curr, 1410) # If faster than 1410 and not boosting, can't speed up
		v_curr = clamp(v_curr + (a_curr * dt), 0, v_curr_max) # v = v + at
		frames += 1
	
	time_elapsed = frames * dt
	v_final = v_curr # TODO: Could return this too
	return time_elapsed


def get_car_strike_loc(ball_loc, packet, car):
	ball_p = packet.game_ball.physics.location
	ball_to_goal = Vec3(0, -team_mult(car.team) * 5120, 642.8) - ball_p
	car_to_ball_2d = (ball_p - car.physics.location).flatten() # ~176 ball center to car joint at diagonal touch
	return ball_loc - ball_to_goal.normal(100) - car_to_ball_2d.normal(84)

def ball_loc_at_time(agent, packet, t): # Time t as seconds_elapsed, clamps to prediction bounds
	bp = agent.ball_prediction # Ball prediction
	t_now = packet.game_info.seconds_elapsed # Current time
	#index = clamp(round((t - t_now) / 60.0), 0, bp.num_slices - 1)
	t_offset = 1.0 / 120.0 # Fake rounding, incase time is between two slices, it picks the closest slice
	for i in range(0, bp.num_slices):
		if t < bp.slices[i].game_seconds + t_offset:
			slice = bp.slices[i]
			#print("i: " + str(i))
			break
	return slice.physics.location

def get_score_total(gtp: GameTickPacket):
	score_total = 0
	for i in range(0, gtp.num_teams):
		score_total += gtp.teams[i].score
	return score_total

# Time when |ball_loc.y| > |y|, good for finding defensive intercept
def ball_past_y_time(agent, packet, y): # Absolute value y coords, returns -1 if |ball.y| not predicted to go above |y|
	y = abs(y)
	bp = agent.ball_prediction # Ball prediction
	t_lowest = -1 # Earliest seconds_elapsed when |ball.y| is past |y|
	for i in range(0, bp.num_slices):
		if abs(bp.slices[i].physics.location.y) > y:
			t_lowest = bp.slices[i].game_seconds
			break
	return t_lowest

def ball_in_goal_time(agent, packet, own=False): # Returns -1 if ball not predicted to go into own net
	bp = agent.ball_prediction # Ball prediction
	t_now = packet.game_info.seconds_elapsed # Current time
	t_lowest = -1 # Earliest seconds_elapsed when ball is "at" the net
	goal_loc = agent.field_info.my_goal.location if own else agent.field_info.opponent_goal.location
	for i in range(0, bp.num_slices):
		slice = bp.slices[i]
		t_slice = slice.game_seconds
		if t_slice < t_now:
			continue
		
		slice_loc = slice.physics.location
		if slice_loc.y * goal_loc.y < 0: # Ball not on proper half
			continue
		if abs(slice_loc.y) > 5215: # Ball "completely" in goal (~5215 from testing, starts entering at ~5030)
			t_lowest = t_slice
			break
	return t_lowest

def ball_in_my_goal_time(agent, packet): # Returns -1 if ball not predicted to go into own net
	return ball_in_goal_time(agent, packet, True)

def ball_in_opponent_goal_time(agent, packet): # Returns -1 if ball not predicted to go into own net
	return ball_in_goal_time(agent, packet, False)

# range(3, 6) is 3, 4, 5

def Get_Ball_At_T(packet, prediction, time):
	start_pred = prediction.slices[0].game_seconds
	current_time = packet.game_info.seconds_elapsed
	delta = current_time - start_pred
	slice_dt = 60 / (prediction.slices[60].game_seconds - start_pred)
	slice = clamp(math.floor((delta + time) * slice_dt), 0, prediction.num_slices - 2)
	s1 = prediction.slices[slice]
	s2 = prediction.slices[slice + 1]
	delta_slice = s2.game_seconds - s1.game_seconds
	return Psuedo_Slice(Psuedo_Physics.lerp(s1.physics, s2.physics, (current_time + time - s1.game_seconds) / delta_slice), current_time + time)

# Distance needed to change velocities (v_i -> v_f) using acceleration a
def accel_dist(v_i, v_f, a): # vf^2 = vi^2 + 2*a*d  ->  d=(vf^2 - vi^2)/(2*a)
	return ((v_f ** 2.0) - (v_i ** 2.0)) / (2.0 * a)
def coast_dist(v_i, v_f=0):
	return accel_dist(v_i, v_f, -525.0) # Coast accel -525.0
def brake_dist(v_i, v_f=0):
	return accel_dist(v_i, v_f, -3500.0) # Brake accel -3500.0

# Gets the boost that we will collect
def get_corner_boost_index(agent):
	
	my_car = agent.packet.game_cars[agent.index]
	
	s = sign(clamp_abs(my_car.physics.location.y, 5000) - agent.field_info.my_goal.location.y)
	s2 = sign(my_car.physics.location.x + my_car.physics.velocity.x * 0.5)
	
	max_l = 0
	max_i = -1
	
	for i, boost in enumerate(agent.field_info.full_boosts):
		if sign(clamp_abs(my_car.physics.location.y + max(0, my_car.physics.velocity.y * s) * s, 5000) - boost.location.y + s * (100 - my_car.boost) * 10) == s and sign(boost.location.x) == s2 and agent.packet.game_boosts[boost.index].is_active:
			if (boost.location.y - agent.field_info.my_goal.location.y) * s > max_l:
				max_i = i
				max_l = (boost.location.y - agent.field_info.my_goal.location.y) * s
		
	
	return max_i
	

def redirect_vect(v1, v2):
	
	# First vector must be flipped
	v1 = v1 * -1
	
	# Normalize vectors
	v1_n = v1.normal()
	v2_n = v2.normal()
	
	# Reflection normal is the vector that points to the midline of these two normalized vectors
	ref = v1_n + (v2_n - v1_n) * 0.5
	
	return ref.normal()

# Keeps track of which cars are dribbling the ball
class Dribble_Tracker:
	def __init__(self, agent):
		self.agent = agent
		self.dribble_timers = []
	
	def set_packet(self, packet):
		ball = packet.game_ball.physics
		
		for i, car in enumerate(packet.game_cars):
			if len(self.dribble_timers) >= i:
				self.dribble_timers.append(0)
			
			# Detects dribble and pushing the ball along the ground.
			if (ball.location - car.physics.location).length() < 260:
				self.dribble_timers[i] += self.agent.delta
			else:
				self.dribble_timers[i] = 0
			
			car.has_dribble = self.dribble_timers[i] > 0.3

class Hit_Package:
	def __init__(self, hit, ground_touch, flip_touch, air_touch):
		self.hit = hit
		self.ground_touch = ground_touch
		self.flip_touch = flip_touch
		self.air_touch = air_touch
	
	def recalculate_time(self, game_time, delta):
		self.hit.recalculate_time(game_time)
		self.ground_touch.recalculate_time(delta)
		self.flip_touch.recalculate_time(delta)
		self.air_touch.recalculate_time(delta)
	
	def to_list(self):
		return [self.hit.to_numpy(), self.ground_touch.to_numpy(), self.flip_touch.to_numpy(), self.air_touch.to_numpy()]
	
	def from_list(_list, game_time):
		return Hit_Package(Simple_Hit_Prediction.from_numpy(_list[0], game_time), Touch.from_numpy(_list[1]), Touch.from_numpy(_list[2]), Touch.from_numpy(_list[3]))

class Path_Hit:
	def __init__(self, drive_path, time):
		self.drive_path = drive_path
		self.time = time

class Path_Vec:
	def get(self, agent, packet, ball):
		raise NotImplementedError

class Shot_On_Goal(Path_Vec):
	def get(self, agent, packet, ball):
		target_vec = (ball.location - agent.field_info.opponent_goal.location)
		target_vec_2 = (ball.location - agent.field_info.opponent_goal.closest_point(ball.location))
		t_v = target_vec_2.copy()
		t_v.y *= 0.3
		lerp_val = clamp((1000 - t_v.length()) / 1000, 0, 1)
		attack_vec = (target_vec_2 * lerp_val + target_vec * (1 - lerp_val)).normal(100 + ball.location.z * 6)
		return attack_vec

class Shot_In_Direction(Path_Vec):
	def __init__(self, vector):
		self.vec = vector * -1
	
	def get(self, agent, packet, ball):
		return self.vec * (100 + ball.location.z * 6)

class Shot_To_Side(Path_Vec):
	def get(self, agent, packet, ball):
		vec = packet.game_cars[agent.index].physics.location - ball.location
		return Vec3(sign(vec.x), -agent.field_info.my_goal.direction.y * 0.1, 0).normal(100 + ball.location.z * 6)

# Calculates the vector needed to hit a shot
def calc_path(path_vec, agent, packet):
	
	my_car = packet.game_cars[agent.index]
	current_t = packet.game_info.seconds_elapsed
	
	target_t = -1
	drive_path = None
	
	for i in range(agent.ball_prediction.num_slices):
		s = agent.ball_prediction.slices[i]
		
		if s.game_seconds - current_t < agent.hit_package.flip_touch.time:
			continue
		
		ball = s.physics
		if ball.location.z < 265 and calc_hit(my_car, ball.location).time < s.game_seconds - current_t:
			attack_vec = path_vec.get(agent, packet, ball)
			dp = Line_Arc_Line(my_car, ball.location + ball.velocity * 0.05 + attack_vec.normal(140), attack_vec)
			if not dp.valid or not dp.check_in_bounds():
				continue
			if dp.calc_time(my_car) < i:
				drive_path = dp
				target_t = i
				break
	
	# Extra refining
	if not drive_path is None:
		i = max(0, target_t - 0.1)
		while i < target_t:
			ball = Get_Ball_At_T(packet, agent.ball_prediction, i).physics
			if ball.location.z < 265 and calc_hit(my_car, ball.location).time < i:
				attack_vec = path_vec.get(agent, packet, ball)
				dp = Line_Arc_Line(my_car, ball.location + attack_vec.normal(140), attack_vec)
				if not dp.valid or not dp.check_in_bounds():
					i += 0.02
					continue
				if dp.calc_time(my_car) < i:
					drive_path = dp
					target_t = i
					break
			i += 0.02
	
	return Path_Hit(drive_path, target_t)
	

class Box:
	def __init__(self, location, dimension):
		self.location = location
		self.dimension = dimension
	
	def point_in_box(self, point):
		return abs(point.x - self.location.x) < self.dimension.x and abs(point.y - self.location.y) < self.dimension.y and abs(point.z - self.location.z) < self.dimension.z
	









