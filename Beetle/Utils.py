import math
import os

from rlbot.utils.structures.game_data_struct import GameTickPacket

from Structs import *

boost_accel = 991.66

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
# Returns the ammount of time needed to drive a distance (len) giving a starting velocity (v) assuming no boost used.
# Uses the above velocity and position tables.
def Time_To_Pos_No_Boost(len, v):
	# Get start "time"
	start = bin_search_velocity(acceleration, v)
	
	# Calculate the end time
	end = bin_search_position(acceleration, start.position + len)
	
	# Add on some additional time for additional length traveled at max speed.
	extra = max(0, start.position + len - end.position) / max(0.01, end.velocity)
	
	# Wow, that was easy :P
	return Hit(end.time - start.time + extra, end.velocity)

boost_consumption = 1/33
# Returns the ammount of time needed for a car to travel a length given a boost ammount and an initial velocity
def Time_to_Pos(len, v, boost):
	t_until_no_boost = boost * boost_consumption
	
	# First, assume unlimited boost
	
	# Get start "time"
	start = bin_search_velocity(boost_acceleration, v)
	
	# Calculate the end time
	end = bin_search_position(boost_acceleration, start.position + len)
	
	# Okay, now let's look at whether we run out of boost. :P
	if end.time - start.time > t_until_no_boost:
		# Oof, okay, let's evaluate our velocity when we run out of boost
		no_boost = bin_search_time(boost_acceleration, start.time + t_until_no_boost)
		if no_boost.velocity > max_car_vel:
			# While technically there is a slight deceleration force applied to the car, for simplicity's sake we will simply assume that the car maintains its current speed
			extra = max(0, start.position + len - no_boost.position) / max(0.01, no_boost.velocity)
			
			# Easy
			return Hit(end.time - start.time + extra, end.velocity)
		else:
			# Okay, so the car will now be accelerating for a bit. Fortunately, we already have a function for this, so we'll just...
			hit = Time_To_Pos_No_Boost(start.position + len - no_boost.position, no_boost.velocity)
			hit.time += no_boost.time - start.time
			return hit
		
	else:
		
		# Add on some additional time for additional length traveled at max speed.
		extra = max(0, start.position + len - end.position) / max(0.01, end.velocity)
		
		# Easy
		return Hit(end.time - start.time + extra, end.velocity)
	

# Needs to be updated to account for added throttle acceleration when < 1410uu
def Time_to_Pos_Old(car, position, velocity, no_correction = False):
	car_to_pos = position - car.physics.location
	
	# Subtract 100 from the length because we contact the ball slightly sooner than we reach the point
	len = max(0, car_to_pos.length() - 200)
	v_len = velocity.length() # * Vec3.dot(car_to_pos.normal(), vel.normal())
	
	# curve:
	# f(t) = 0.5 * boost_accel * t ^ 2 + velocity * t
	
	# Analysis of t:
	# Solve for t when f(t) = len
	# Zeros of t: let c = len
	# 0.5 * boost_accel * t ^ 2 + velocity * t - len = 0
	
	# t = ( -velocity + sqrt(velocity^2 - 4(boost_accel)(-len)) ) / ( 2 * (boost_accel) )
	
	accel_time = (-v_len + math.sqrt(v_len * v_len + 4 * boost_accel * len)) / (2 * boost_accel)
	
	# However, the car speed maxes out at 2300 uu, so we need to account for that by stopping acceleration at 2300 uu. To do this we
	# calculate when we hit 2300 uu and cancel out any acceleration that happens after that
	
	# f(t) = 0.5 * boost_accel * t ^ 2 + velocity * t
	# Derivative:
	# v(t) = boost_accel * t + velocity
	# Solve for t when v(t) = 2300
	# 2300 = boost_accel * t + velocity
	# 2300 - velocity = boost_accel * t
	# ( 2300 - velocity ) / boost_accel = t
	
	max_vel_time = (2300 - v_len) / boost_accel
	
	a = 0
	b = 0
	
	if max_vel_time < accel_time:
		
		# plug time into position function
		pos = 0.5 * boost_accel * max_vel_time * max_vel_time + v_len * max_vel_time
		
		# Calculate additional distance that needs to be traveled
		extra_vel = len - pos
		
		# Add additional time onto velocity
		a = max_vel_time + extra_vel / 2300
		
		b = 2300
		
	else:
		a = accel_time
		
		b = 0.5 * boost_accel * a * a + v_len * a
	
	if not no_correction:
		# Finally, we account for higher values being, well, higher. Not an exact science, but...
		a = (1 + car_to_pos.z * 0.004)
	
	return Hit(a, b)
	

class Touch():
	def __init__(self, time, location, is_garunteed = False, can_save = True):
		self.time = time
		self.location = location
		self.is_garunteed = is_garunteed
		self.can_save = can_save

# Predictions from this will be a little off. Need to make it take into account the change of position in the turn
class Hit_Prediction():
	def __init__(self, agent, packet):
		self.prediction = agent.ball_prediction
		team = agent.team
		current_time = packet.game_info.seconds_elapsed
		self.hit_time = 0
		for i in range(0, self.prediction.num_slices, 6):
			slice = self.prediction.slices[i]
			if slice.game_seconds < current_time:
				continue
			
			for car in packet.game_cars:
				if car.team == team or car.is_demolished:
					continue
				
				loc = slice.physics.location
				t = slice.game_seconds - current_time
				if loc.z > 200:
					air_hit = self.calc_air(car, loc, t, packet.game_info.world_gravity_z)
					if (air_hit.velocity.length() < 850 and t < car.boost * (1/33)):
						self.hit_time = slice.game_seconds - current_time
						self.hit_game_seconds = slice.game_seconds
						self.hit_position = loc
						self.hit_velocity = 1500 # Dummy value for now :P
				else:
					hit = self.calc_hit(car, loc)
					if hit.time < slice.game_seconds - current_time:
						# print(slice.game_seconds - current_time)
						self.hit_time = slice.game_seconds - current_time
						self.hit_game_seconds = slice.game_seconds
						self.hit_position = loc
						self.hit_velocity = min(2400, hit.velocity + 500) # Add 500 for flipping (Need to update to take into account direction of hit)
			
			if not self.hit_time and i + 6 >= self.prediction.num_slices:
				self.hit_time = 6
				self.hit_position = self.prediction.slices[i].physics.location
				self.hit_game_seconds = current_time + 6
				self.hit_velocity = 0
			
			if self.hit_time:
				break
	
	def draw_path(self, renderer, c):
		poly = []
		for i in range(0, self.prediction.num_slices, 5):
			slice = self.prediction.slices[i]
			if slice.game_seconds > self.hit_game_seconds:
				break
			poly.append(slice.physics.location.UI_Vec3())
		renderer.draw_polyline_3d(poly, c)
		renderer.draw_line_3d(poly[len(poly) - 1], self.hit_position.UI_Vec3(), c)
	
	# Determines if a car can hit a position at a time. Designed for ground play.
	def calc_hit(self, car, position):
		car_vel = car.physics.velocity
		car_vel_len = car_vel.length()
		car_pos = car.physics.location
		car_face = Vec3(1, 0, 0).align_to(car.physics.rotation)
		
		# Calculate the length of the car's path
		car_path = (position - car_pos).flatten()
		angle = car_vel.angle_between(car_path) #TODO: SHOULD BE TIMES 2?!
		
		# Length of the path the car must travel
		turn = turn_radius(car_vel_len) * angle
		len = car_path.length()
		
		# Calculate the time to get to the position
		turn_time = turn / max(500, car_vel_len)
		drive_time = Time_to_Pos(max(0.01, len - 200), car_vel_len, car.boost) if car_face.dot(car_vel) > 0.0 else Time_To_Pos_No_Boost(max(0.01, (len - 200)), car_vel_len)
		
		drive_time.time += turn_time # + air_time
		
		return drive_time
	
	# Designed for aerials. Calculates the delta v to hit a location at a time.
	def calc_air(self, car, position, time, grav_z):
		dv = delta_v(car, position, max(0.0001, time), grav_z, car.physics.velocity + Vec3(0, 0, 300 if car.has_wheel_contact else 0).align_to(car.physics.rotation))
		
		return Hit(velocity = dv)
	
	# Return the earliest touch time along with a flag for viability
	def get_earliest_touch(self, agent, packet, car, max_height = 4000, offset = None):
		if offset is None:
			offset = Vec3()
		current_time = packet.game_info.seconds_elapsed
		goal_pos = agent.field_info.my_goal.location
		goal_pos.z = min(goal_pos.z, max_height)
		goal_pos.x = clamp_abs(self.hit_position.x, 650)
		goal_vec = (goal_pos - self.hit_position).normal(self.hit_velocity)
		for i in range(0, self.prediction.num_slices, 3):
			slice = self.prediction.slices[i]
			if slice.game_seconds < current_time:
				continue
			loc = (slice.physics.location if slice.game_seconds < self.hit_game_seconds else self.hit_position + goal_vec * (slice.game_seconds - self.hit_game_seconds)) + offset
			t = slice.game_seconds - current_time
			if loc.z > 200:
				air_hit = self.calc_air(car, loc, slice.game_seconds - current_time, packet.game_info.world_gravity_z)
				if (air_hit.velocity.length() < 850 and t < car.boost * 0.033 and loc.z < max_height) or i >= self.prediction.num_slices - 3 or abs(loc.y) > 5120:
					return Touch(t, loc, t <= self.hit_time, abs(loc.y) > 5120)
			else:
				#car_strike_loc = get_car_strike_loc(loc, packet, car)
				#hit = self.calc_hit(car, car_strike_loc)
				hit = self.calc_hit(car, loc)
				if (hit.time < slice.game_seconds - current_time and loc.z < max_height) or i >= self.prediction.num_slices - 3 or abs(loc.y) > 5120:
					return Touch(max(t, hit.time), loc, t <= self.hit_time, abs(loc.y) > 5120)
		
		return Touch(6, goal_pos, False, True)
	

def render_star(self, position: Vec3, color, size = 100):
	
	v = Vec3(1, 1, 1).normal(size)
	
	self.renderer.draw_line_3d((position - Vec3(size, 0, 0)).UI_Vec3(), (position + Vec3(size, 0, 0)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(0, size, 0)).UI_Vec3(), (position + Vec3(0, size, 0)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(0, 0, size)).UI_Vec3(), (position + Vec3(0, 0, size)).UI_Vec3(), color)
	
	self.renderer.draw_line_3d((position - Vec3(-v.x, v.y, v.z)).UI_Vec3(), (position + Vec3(-v.x, v.y, v.z)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(v.x, -v.y, v.z)).UI_Vec3(), (position + Vec3(v.x, -v.y, v.z)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(v.x, v.y, -v.z)).UI_Vec3(), (position + Vec3(v.x, v.y, -v.z)).UI_Vec3(), color)
	

def delta_v(car, position, time, grav_z, car_vel = None):
	if not car_vel:
		car_vel = car.physics.velocity
	car_pos = car.physics.location
	return Vec3(
		(position.x - car_vel.x * time - car_pos.x) / (0.5 * time * time),
		(position.y - car_vel.y * time - car_pos.y) / (0.5 * time * time),
		(position.z - car_vel.z * time - car_pos.z) / (0.5 * time * time) - grav_z,
	)

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
	delta = packet.game_info.seconds_elapsed - prediction.slices[0].game_seconds
	return prediction.slices[clamp(math.floor((delta + time) * 60), 0, prediction.num_slices - 1)]


# Distance needed to change velocities (v_i -> v_f) using acceleration a
def accel_dist(v_i, v_f, a): # vf^2 = vi^2 + 2*a*d  ->  d=(vf^2 - vi^2)/(2*a)
	return ((v_f ** 2.0) - (v_i ** 2.0)) / (2.0 * a)
def coast_dist(v_i, v_f=0):
	return accel_dist(v_i, v_f, -525.0) # Coast accel -525.0
def brake_dist(v_i, v_f=0):
	return accel_dist(v_i, v_f, -3500.0) # Brake accel -3500.0


















