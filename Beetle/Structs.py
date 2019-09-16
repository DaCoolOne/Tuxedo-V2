import math

from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.structures.game_data_struct import Vector3 as UI_Vec3

from enum import Enum

def sign(n):
	return 1 if n > 0 else -1

def constrain(n):
	return max(-1, min(1, n))

class Vec3:
	def __init__(self, x=0, y=0, z=0):
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)
	
	def __add__(self, val):
		return Vec3(self.x + val.x, self.y + val.y, self.z + val.z)
	
	def __sub__(self, val):
		return Vec3(self.x - val.x, self.y - val.y, self.z - val.z)
	
	def __mul__(self, val):
		return Vec3(self.x * val, self.y * val, self.z * val)
	
	def length(self):
		return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
	
	def set(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
	
	def align_to(self, rot):
		v = Vec3(self.x, self.y, self.z)
		v.set(v.x, math.cos(rot.roll) * v.y + math.sin(rot.roll) * v.z, math.cos(rot.roll) * v.z - math.sin(rot.roll) * v.y)
		v.set(math.cos(-rot.pitch) * v.x + math.sin(-rot.pitch) * v.z, v.y, math.cos(-rot.pitch) * v.z - math.sin(-rot.pitch) * v.x)
		v.set(math.cos(-rot.yaw) * v.x + math.sin(-rot.yaw) * v.y, math.cos(-rot.yaw) * v.y - math.sin(-rot.yaw) * v.x, v.z)
		return v
	
	def align_from(self, rot):
		v = Vec3(self.x, self.y, self.z)
		v.set(math.cos(rot.yaw) * v.x + math.sin(rot.yaw) * v.y, math.cos(rot.yaw) * v.y - math.sin(rot.yaw) * v.x, v.z)
		v.set(math.cos(rot.pitch) * v.x + math.sin(rot.pitch) * v.z, v.y, math.cos(rot.pitch) * v.z - math.sin(rot.pitch) * v.x)
		v.set(v.x, math.cos(-rot.roll) * v.y + math.sin(-rot.roll) * v.z, math.cos(-rot.roll) * v.z - math.sin(-rot.roll) * v.y)
		return v
	
	def UI_Vec3(self):
		return UI_Vec3(self.x, self.y, self.z)
	
	def copy(self):
		return Vec3(self.x, self.y, self.z)
	
	def flatten(self):
		return Vec3(self.x, self.y, 0.0)
	
	def normal(self, n = 1):
		l = max(self.length(), 0.0001)
		return Vec3(self.x / l * n, self.y / l * n, self.z / l * n)
	
	def tostring(self):
		return str(self.x) + "," + str(self.y) + "," + str(self.z)
	
	def cast(v):
		return Vec3(float(v.x), float(v.y), float(v.z))
	
	def dot(v1, v2):
		return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z
	
	# Returns the angle between two vectors
	def angle_between(v1, v2):
		return math.acos(v1.normal().dot(v2.normal()))
	
	def angle_2d(self):
		return math.atan2(self.y, self.x)

class Rotation():
	def __init__(self, rot):
		self.yaw = rot.yaw
		self.pitch = rot.pitch
		self.roll = rot.roll
	
	# Returns the angle between two rotations
	def angle_between(r1, r2):
		return math.acos(Vec3.dot(Vec3(1, 0, 0).align_to(r1), Vec3(1, 0, 0).align_to(r2)))
	
	# Returns the angle between a rotation and a vector
	def angle_to_vec(r, v):
		return math.acos(Vec3.dot(Vec3(1, 0, 0).align_to(r), v.normal()))
	
	def copy(self):
		return Rotation(self)
	

class Hitbox():
	def __init__(self,_list):
		self.length = _list.length
		self.width = _list.width
		self.height = _list.height


	def get_offset_by_angle(self,angleDegrees):
		if abs(angleDegrees) <=45:
			return self.length*.666
		elif abs(angleDegrees) >= 135:
			return self.length*.333
		else:
			return self.width*.5

class Physics():
	def __init__(self, physics):
		self.location = Vec3.cast(physics.location)
		self.velocity = Vec3.cast(physics.velocity)
		self.rotation = Rotation(physics.rotation)
		self.angular_velocity = Vec3.cast(physics.angular_velocity)
	

class Psuedo_Physics():
	def __init__(self, location = None, velocity = None, rotation = None, angular_velocity = None):
		self.location = location
		self.velocity = velocity
		self.rotation = rotation
		self.angular_velocity = angular_velocity
	
	def lerp(p1, p2, s):
		s2 = 1 - s
		return Psuedo_Physics(
			p1.location * s2 + p2.location * s,
			p1.velocity * s2 + p2.velocity * s,
			p1.rotation, # Todo: Figure out how to lerp rotations
			p1.angular_velocity * s2 + p2.angular_velocity * s
		)
	
	def cast(self, p):
		return Psuedo_Physics(p.location.copy(), p.velocity.copy(), p.rotation.copy(), p.angular_velocity.copy())

class LatestTouch():
	def __init__(self, touch):
		self.player_index = touch.player_index
		self.hit_location = Vec3.cast(touch.hit_location)
		self.hit_normal = Vec3.cast(touch.hit_normal)
		self.team = touch.team
		self.time_seconds = touch.time_seconds
	

class GameInfo():
	def __init__(self, info):
		self.seconds_elapsed = info.seconds_elapsed
		self.game_time_remaining = info.game_time_remaining
		self.is_overtime = info.is_overtime
		self.is_unlimited_time = info.is_unlimited_time
		self.is_round_active = info.is_round_active
		self.is_kickoff_pause = info.is_kickoff_pause
		self.world_gravity_z = info.world_gravity_z
	

class Ball():
	def __init__(self, game_ball):
		self.physics = Physics(game_ball.physics)
		self.latest_touch = LatestTouch(game_ball.latest_touch)
	

class Car():
	def __init__(self, car):
		self.physics = Physics(car.physics)
		self.is_demolished = car.is_demolished
		self.has_wheel_contact = car.has_wheel_contact
		self.is_super_sonic = car.is_super_sonic
		self.jumped = car.jumped
		self.double_jumped = car.double_jumped
		self.team = car.team
		self.boost = car.boost
		self.has_dribble = False
		self.hitbox = Hitbox(car.hitbox)

class Boost():
	def __init__(self, boost):
		self.is_active = boost.is_active
		self.time = boost.timer

class Team():
	def __init__(self, team):
		self.team_index = team.team_index
		self.score = team.score

class Packet():
	def __init__(self, packet):
		self.game_ball = Ball(packet.game_ball)
		self.game_info = GameInfo(packet.game_info)
		self.game_cars = []
		for i in range(packet.num_cars):
			self.game_cars.append(Car(packet.game_cars[i]))
		self.game_boosts = []
		for i in range(packet.num_boost):
			self.game_boosts.append(Boost(packet.game_boosts[i]))
		self.teams = []
		for i in range(packet.num_teams):
			self.teams.append(Team(packet.teams[i]))
	

class GoalInfo():
	def __init__(self, goal):
		self.location = Vec3.cast(goal.location)
		self.direction = Vec3.cast(goal.direction)
	
	# Returns the closest shootable point in the goal
	def closest_point(self, position, width = 800):
		p = min(width, abs(self.location.x - position.x)) * sign(position.x - self.location.x)
		v = self.location.copy()
		v.x = self.location.x + p
		return v

class BoostInfo():
	def __init__(self, boost, index):
		self.location = Vec3.cast(boost.location)
		self.is_full_boost = boost.is_full_boost
		self.index = index
	

class FieldInfo():
	def __init__(self, agent, field_info):
		self.full_boosts = []
		self.boosts = []
		self.my_goal = None
		self.opponent_goal = None
		for i in range(field_info.num_goals):
			goal = field_info.goals[i]
			if goal.team_num == agent.team:
				self.my_goal = GoalInfo(goal)
			else:
				self.opponent_goal = GoalInfo(goal)
		for i in range(field_info.num_boosts):
			boost = BoostInfo(field_info.boost_pads[i], i)
			self.boosts.append(boost)
			if boost.is_full_boost:
				self.full_boosts.append(boost)

class Slice:
	def __init__(self, s):
		self.physics = Physics(s.physics)
		self.game_seconds = s.game_seconds

class Psuedo_Slice:
	def __init__(self, physics, game_seconds):
		self.physics = physics
		self.game_seconds = game_seconds
	
	def cast(slice):
		p = Psuedo_Physics.cast(slice.physics)
		return Psuedo_Slice(p, slice.game_seconds)

# Only take every third ball slice because it's faster
class BallPrediction:
	def __init__(self, prediction):
		self.slices = []
		self.num_slices = int(prediction.num_slices / 3)
		for i in range(0, prediction.num_slices, 3):
			self.slices.append(Slice(prediction.slices[i]))
	

class MyControllerState():
	def __init__(self):
		self.throttle = 0
		self.boost = 0
		self.handbrake = 0
		self.jump = 0
		self.steer = 0
		self.pitch = 0
		self.yaw = 0
		self.roll = 0
	
	def get(self):
		return SimpleControllerState(
			constrain(self.steer),
			constrain(self.throttle),
			constrain(self.pitch),
			constrain(self.yaw),
			constrain(self.roll),
			self.jump,
			self.boost,
			self.handbrake,
		)

class TouchType(Enum):
	ground = 0
	flip = 1
	aerial = 2
