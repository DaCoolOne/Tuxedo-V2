from Utils import *

import math

from enum import Enum

class Maneuver:
	def update(self, agent, packet):
		pass

class Maneuver_Flip(Maneuver):
	def __init__(self, agent, packet, direction, time = 0.8):
		self.direction = direction
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.max_time = time
		self.f = Vec3(1, 0, 0).align_to(packet.game_cars[agent.index].physics.rotation)
	
	def update(self, agent, packet):
		dt = packet.game_info.seconds_elapsed - self.start_time
		if dt < 0.15:
			# Jump into air
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = True
			agent.controller_state.throttle = self.direction.x
		elif dt < 0.3:
			# Pause
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = False
			agent.controller_state.throttle = 0
		elif not self.has_flipped:
			# Perform the flip
			agent.controller_state.jump = True
			agent.controller_state.pitch = -self.direction.x
			agent.controller_state.roll = self.direction.y
			self.has_flipped = True
		else:
			agent.controller_state.jump = False
			# Return control back to the agent
			return dt > self.max_time
		return False


class Precise_Maneuver_Flip(Maneuver):
	def __init__(self, agent, packet, direction, time=0.8):
		self.direction = direction
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.max_time = time
		self.f = Vec3(1, 0, 0).align_to(packet.game_cars[agent.index].physics.rotation)

	def update(self, agent, packet):
		dt = packet.game_info.seconds_elapsed - self.start_time
		if dt < 0.15:
			# Jump into air
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = True
			agent.controller_state.throttle = self.direction.x
		elif dt < 0.3:
			# Pause
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = False
			agent.controller_state.throttle = 0
		elif not self.has_flipped:
			# Perform the flip
			agent.controller_state.jump = True
			agent.controller_state.pitch = -self.direction.x
			agent.controller_state.roll = self.direction.y
			self.has_flipped = True
		else:
			agent.controller_state.jump = False
			# Return control back to the agent
			return dt > self.max_time
		return False

class Maneuver_Half_Flip(Maneuver):
	def __init__(self, agent, packet):
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.f = Vec3(1, 0, 0).align_to(packet.game_cars[agent.index].physics.rotation)
	
	def update(self, agent, packet):
		dt = packet.game_info.seconds_elapsed - self.start_time
		if dt < 0.1:
			# Jump into air
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = True
			agent.controller_state.throttle = -1
		elif dt < 0.2:
			# Pause
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = False
			agent.controller_state.throttle = 0
		elif not self.has_flipped:
			# Perform the flip
			agent.controller_state.jump = True
			agent.controller_state.pitch = 1
			agent.controller_state.roll = 0
			self.has_flipped = True
		else:
			Align_Car_To(agent, packet, self.f * -1, Vec3(0, 0, 1))
		return dt > 1.2

class Maneuver_Flip_Ball(Maneuver):
	def __init__(self, agent, packet, time = 0.8):
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.max_time = time
		self.f = Vec3(1, 0, 0).align_to(packet.game_cars[agent.index].physics.rotation)
	
	def update(self, agent, packet):
		dt = packet.game_info.seconds_elapsed - self.start_time
		if dt < 0.2:
			# Jump into air
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			agent.controller_state.jump = True
			car = packet.game_cars[agent.index]
			agent.controller_state.throttle = (car.physics.velocity).align_from(car.physics.rotation).x
		elif dt < 0.25:
			Align_Car_To(agent, packet, self.f, Vec3(0, 0, 1))
			# Pause
			agent.controller_state.jump = False
			agent.controller_state.throttle = 0
		elif not self.has_flipped:
			# Perform the flip
			my_car = packet.game_cars[agent.index]
			vec = packet.game_ball.physics.location - my_car.physics.location
			vec = vec + packet.game_ball.physics.velocity.normal(vec.length() * 0.5)
			direction = vec.align_from(my_car.physics.rotation).flatten().normal()
			agent.controller_state.jump = True
			agent.controller_state.pitch = -direction.x
			agent.controller_state.roll = direction.y
			self.has_flipped = True
		else:
			agent.controller_state.jump = False
			# Return control back to the agent
			return dt > self.max_time
		return False

class Maneuver_Jump_Shot(Maneuver):
	def __init__(self, agent, packet, intersect_time, target):
		self.start_time = packet.game_info.seconds_elapsed
		self.target = target
		self.delay = clamp(intersect_time, 0.3, 1.25)
		if self.delay >= 0.3:
			if target.z <= 200:
				self.jumpTimerMax = 0.1
				self.angleTimer = clamp(self.jumpTimerMax / 4,0.05,0.15)
			else:
				self.jumpTimerMax = self.delay - 0.2
				self.angleTimer = clamp(self.jumpTimerMax / 4, 0.1, 0.15)
		self.jumped = False
		self.jumpTimer = 0

	def update(self,agent, packet):
		age = packet.game_info.seconds_elapsed - self.start_time
		controller_state = agent.controller_state
		controller_state.throttle = 0
		controller_state.boost = False
		car = packet.game_cars[agent.index]
		position = Vec3_from_Vector3(car.physics.location)
		if not self.jumped:
			self.jumped = True
			controller_state.jump = True
		else:
			#jumpTimer = age

			if age < self.angleTimer:
				Align_Car_To(agent, packet,(position-self.target).normal(), Vec3(0, 0, 1))

			if age < self.jumpTimerMax:
				controller_state.jump = True

			else:
				if age >= self.jumpTimerMax:
					if age >= self.delay - 0.2 and age < self.delay - 0.15:
						controller_state.jump = False
					elif age >= self.delay - 0.15 and age < self.delay:
						vec = self.target - car.physics.location
						direction = vec.align_from(car.physics.rotation).normal()
						agent.controller_state.jump = True
						agent.controller_state.pitch = -direction.x
						agent.controller_state.roll = direction.y
					elif age < self.delay + 0.1:
						controller_state.jump = False
					else:
						controller_state.jump = False
						return True
		return False

class Maneuver_Flick(Maneuver):
	def __init__(self, packet, direction, time = 0.8):
		self.direction = direction
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.max_time = time
	
	def update(self, agent, packet):
		dt = packet.game_info.seconds_elapsed - self.start_time
		if dt < 0.05:
			agent.controller_state.throttle = 0
		elif dt < 0.3:
			# Jump into air
			agent.controller_state.jump = True
			agent.controller_state.throttle = 0
			agent.controller_state.pitch = 1 # Pitch the nose back for the flick (experimental)
		elif dt < 0.35:
			# Pause
			agent.controller_state.jump = False
			agent.controller_state.throttle = 0
		elif not self.has_flipped:
			# Perform the flip
			agent_car = packet.game_cars[agent.index]
			self.direction = (agent_car.physics.location - packet.game_ball.physics.location).align_from(agent_car.physics.rotation)
			
			self.direction.y = self.direction.y * 0.5
			self.direction = self.direction.flatten().normal()
			
			agent.controller_state.jump = True
			agent.controller_state.pitch = self.direction.x
			agent.controller_state.roll = self.direction.y
			self.has_flipped = True
		else:
			agent.controller_state.jump = False
			# Return control back to the agent
			return dt > self.max_time
		return False

class SpinJump(Maneuver):
	def __init__(self, direction):
		self.direction = direction
		self.has_jumped = False
	
	def update(self, agent, packet):
		my_car = packet.game_cars[agent.index]
		Align_Car_To(agent, packet, self.direction, Vec3(0, 0, 1))
		agent.controller_state.pitch = 0
		agent.controller_state.jump = not self.has_jumped
		self.has_jumped = self.has_jumped or not my_car.has_wheel_contact
		return my_car.has_wheel_contact and my_car.physics.velocity.z < 10 and self.has_jumped

class Demo_Dodge(SpinJump):
	def __init__(self, agent, packet, car):
		super().__init__(Vec3(1, 0, 0).align_to(packet.game_cars[agent.index].physics.rotation))
		self.has_jumped = True
		agent.controller_state.jump = True

def Avoid_Demo(agent, packet, car):
	agent.maneuver = Demo_Dodge(agent, packet, car)
	agent.maneuver_complete = False

def Enter_Flip(agent, packet, direction, low_flip = False, control_time = None):
	if control_time:
		agent.maneuver = Maneuver_Flip(agent, packet, direction, control_time)
	else:
		agent.maneuver = Maneuver_Flip(agent, packet, direction)
	agent.maneuver_complete = False
	if low_flip:
		agent.maneuver.start_time -= 0.1

def Flip_To_Ball(agent, packet, low_flip = False):
	agent.maneuver = Maneuver_Flip_Ball(agent, packet)
	agent.maneuver_complete = False
	if low_flip:
		agent.maneuver.start_time -= 0.1

def Enter_Flick(agent, packet, direction):
	agent.maneuver = Maneuver_Flick(packet, direction)
	agent.maneuver_complete = False

min_heading_err_for_handbrake = 100 #90?
def Align_Car_To(self, packet, vector: Vec3, up = Vec3(0, 0, 0)):
	
	my_car = packet.game_cars[self.index]
	
	car_rot = my_car.physics.rotation
	
	car_rot_vel = my_car.physics.angular_velocity
	
	local_euler = car_rot_vel.align_from(car_rot)
	
	align_local = vector.align_from(car_rot)
	
	local_up = up.align_from(car_rot)
	
	# Improving this
	rot_ang_const = 0.25
	stick_correct = 6.0
	
	a1 = math.atan2(align_local.y, align_local.x)
	a2 = math.atan2(align_local.z, align_local.x)
	
	if local_up.y == 0 and local_up.z == 0:
		a3 = 0.0
	else:
		a3 = math.atan2(local_up.y, local_up.z)
	
	yaw = correct(0.0, -a1 + local_euler.z * rot_ang_const, stick_correct)
	pitch = correct(0.0, -a2 - local_euler.y * rot_ang_const, stick_correct)
	roll = correct(0.0, -a3 - local_euler.x * rot_ang_const, stick_correct)
	
	self.controller_state.yaw = constrain(yaw)
	self.controller_state.pitch = constrain(pitch)
	self.controller_state.roll = constrain(roll)
	
	self.controller_state.steer = constrain(yaw)

def drive(agent, packet, target_loc, time_allotted, target_v=-1, min_straight_spd=0, always_boost=False, allow_flips=False):
	target_loc = target_loc.copy()
	
	car = packet.game_cars[agent.index]
	car_p = car.physics.location # Car position
	car_v = car.physics.velocity.flatten() # Car velocity
	car_dir = Vec3(1, 0, 0).align_to(car.physics.rotation) # Car direction
	
	on_ground = abs(car_dir.z) < 0.1
	
	arc_turn = ArcTurn(car_p, car_dir, target_loc)
	if arc_turn.valid:
		arc_turn.render(agent.renderer, agent.renderer.blue())
	
	max_turn = turn_radius(car_v.length())
	
	if abs(car_p.x) < 885 and abs(car_p.y) > 5050 and abs(target_loc.x) > 800: # Car in net
		target_loc.x = 800 * sign(target_loc.x)
	
	car_to_loc_3d = target_loc.flatten() - car_p
	car_to_loc = car_to_loc_3d.flatten() # - car_dir * 40
	target_speed = car_to_loc.length() / max(0.001, time_allotted) #TODO: Handle divide by zero
	current_speed = car_v.length() * sign(Vec3.dot(car_to_loc, car_dir))
	
	#delta_speed = (target_speed - current_speed) * 0.01 - 0.1
	speed_err = current_speed - target_speed # >0 -> bot going too fast
	heading_err = correction(car, car_to_loc_3d)
	heading_err_deg_abs = abs(math.degrees(heading_err))
	
	# Half flips
	if heading_err_deg_abs > 150 and allow_flips and car_to_loc.length() > 500 and car_v.length() < 200:
		agent.maneuver = Maneuver_Half_Flip(agent, packet)
		agent.maneuver_complete = False
		return MyControllerState()
	
	cs = MyControllerState()
	cs.steer = steer_for_heading_err(heading_err)
	cs.handbrake = car_v.length() > 500 and (max_turn > arc_turn.radius or (arc_turn.arc_length / car_v.length() > time_allotted and heading_err_deg_abs > 60) and time_allotted > 0.2) and on_ground and car_v.dot(car_dir) > 0
	
	if target_speed < min_straight_spd and heading_err_deg_abs < 10:
		cs.throttle = 0.0
		cs.boost = False
	elif speed_err > 0:
		if speed_err > 10:
			cs.throttle = 0.0
		else:
			cs.throttle = 0.02
		cs.boost = False
	else: # <0
		cs.throttle = 1 #constrain(-speed_err * 0.01 - 0.1)
		should_boost = current_speed < 2275 and (always_boost or speed_err < (-500 / max(0.05, time_allotted)) or (current_speed > 1410 and speed_err < -1))
		cs.boost = should_boost and not cs.handbrake
	
	dist_until_brake = car_to_loc.length() - brake_dist(car_v.length(), target_v) - (2.0 * car_v.length() * (1.0 / 60.0))
	if target_v != -1 and car_v.length() > target_v - 10 and dist_until_brake < 0: # brake
		cs.throttle = -1
		cs.boost = False
	
	if speed_err < -1000 and car_v.length() > 1250 and car_to_loc.length() > car_v.length() * (1.5 if target_v == -1 else 3) and allow_flips and Vec3(1, 0, 0).align_to(car.physics.rotation).dot(car_to_loc.normal()) > 0.95:
		Enter_Flip(agent, packet, Vec3(1, 0, 0))
	
	# Demo avoidance
	if time_allotted > 1.0:
		for other_car in packet.game_cars:
			if ((other_car.physics.velocity.length() > 1800 and other_car.team != agent.team and (other_car.physics.location + other_car.physics.velocity * 0.5 - car.physics.location - car.physics.velocity * 0.5).length() < 300)):
				Enter_Flip(agent, packet, Vec3(1, 0, 0), low_flip = True)
				break
	
	return cs


def drive_catch(agent, packet, target_loc, time_allotted, allow_flips = False):
	car = packet.game_cars[agent.index]
	car_p = car.physics.location # Car position
	car_v = car.physics.velocity.flatten() # Car velocity
	car_dir = Vec3(1, 0, 0).align_to(car.physics.rotation) # Car direction
	
	if abs(car_p.x) < 885 and abs(car_p.y) > 5050: # Car in net
		target_loc.x = 800 * sign(target_loc.x)
	
	car_to_loc_3d = target_loc.flatten() - car_p
	car_to_loc = car_to_loc_3d.flatten() # - car_dir * 40
	target_speed = car_to_loc.length() / max(0.001, time_allotted) #TODO: Handle divide by zero
	current_speed = car_v.length()
	
	#delta_speed = (target_speed - current_speed) * 0.01 - 0.1
	speed_err = current_speed - target_speed # >0 -> bot going too fast
	heading_err = correction(car, car_to_loc_3d)
	heading_err_deg_abs = abs(math.degrees(heading_err))
	
	# Half flips
	if heading_err_deg_abs > 160 and allow_flips and car_to_loc.length() > 500 and car_v.length() < 200:
		agent.maneuver = Maneuver_Half_Flip(agent, packet)
		agent.maneuver_complete = False
		return MyControllerState()
	
	cs = MyControllerState()
	cs.steer = steer_for_heading_err(heading_err)
	cs.handbrake = car_v.length() > 400 and heading_err_deg_abs > min_heading_err_for_handbrake and car_to_loc.length() < car_v.length()
	
	cs.throttle = -speed_err * 0.01 - 0.5
	should_boost = current_speed < 2300 and (speed_err < (-700 / max(0.2, time_allotted)) or (current_speed > 1410 and speed_err < -1))
	cs.boost = should_boost and not cs.handbrake
	
	# if target_speed < min_straight_spd and heading_err_deg_abs < 10:
		# cs.throttle = 0.0
		# cs.boost = False
	# elif speed_err > 0:
		# if speed_err > 40:
			# cs.throttle = 0.0
		# else:
			# cs.throttle = 0.02
		# cs.boost = False
	# else: # <0
		# cs.throttle = 0 #constrain(-speed_err * 0.01 - 0.1)
		# should_boost = current_speed < 2300 and (always_boost or speed_err < (-700 / max(0.2, time_allotted)) or (current_speed > 1410 and speed_err < -1))
		# cs.boost = should_boost and not cs.handbrake
	
	# Todo, adjust these parameters
	if speed_err < -1000 and car_v.length() > 1250 and car_to_loc.length() > car_v.length() * 1.5 and allow_flips and Vec3(1, 0, 0).align_to(car.physics.rotation).dot(car_to_loc.normal()) > 0.95:
		Enter_Flip(agent, packet, Vec3(1, 0, 0))
	
	return cs

def JumpShot(agent, position:Vec3):
	my_car = agent.packet.game_cars[agent.index]

	hit = Hit_Prediction(agent, agent.packet)
	touch = hit.get_earliest_touch(agent, agent.packet, my_car)

	v1 = touch.location - position
	vec = (v1 + packet.game_ball.physics.velocity.normal(v1.length() * 0.5)) * 3
	if vec.length() > 15:
		vec = vec.normal(15)

	# ball_pos = packet.game_ball.physics.location + packet.game_ball.physics.velocity * 0.2 - vec

	# car_to_ball = ball_pos - my_car.physics.location

	self.controller_state = drive(self, packet,
								  touch.location + vec - Vec3(20, 0, 100).align_to(my_car.physics.rotation),
								  touch.time * 0.9)

	# self.controller_state.steer = -correction(my_car, car_to_ball)
	# self.controller_state.throttle = car_to_ball.len() * 0.3

	if (pos(my_car) - packet.game_ball.physics.location).length() < 200:
		for car in packet.game_cars:
			l = car.physics.location + vel(car) * 0.5 - pos(my_car) - vel(my_car) * 0.5
			self.renderer.draw_line_3d(pos(car).UI_Vec3(), (pos(car) + vel(car) * 0.5).UI_Vec3(), self.renderer.white())
			if (
					l.length() < 500.0 and self.team != car.team and not car.is_demolished and car.physics.location.z + car.physics.velocity.z * 0.4 < 280.0):
				Enter_Flick(self, packet, Vec3(1, 0, 0))
				return


def Dribble(self, packet, position: Vec3):
	
	my_car = packet.game_cars[self.index]
	
	hit = Hit_Prediction(self, self.packet)
	touch = hit.get_earliest_touch(self, self.packet, my_car)
	
	v1 = touch.location - position
	vec = (v1 + packet.game_ball.physics.velocity.normal(v1.length() * 0.5)) * 3
	if vec.length() > 15:
		vec = vec.normal(15)
	
	# ball_pos = packet.game_ball.physics.location + packet.game_ball.physics.velocity * 0.2 - vec
	
	# car_to_ball = ball_pos - my_car.physics.location
	
	self.controller_state = drive(self, packet, touch.location + vec - Vec3(20, 0, 100).align_to(my_car.physics.rotation), touch.time * 0.9)
	
	# self.controller_state.steer = -correction(my_car, car_to_ball)
	# self.controller_state.throttle = car_to_ball.len() * 0.3
	
	if (pos(my_car) - packet.game_ball.physics.location).length() < 200:
		for car in packet.game_cars:
			l = car.physics.location + vel(car) * 0.5 - pos(my_car) - vel(my_car) * 0.5
			self.renderer.draw_line_3d(pos(car).UI_Vec3(), (pos(car) + vel(car) * 0.5).UI_Vec3(), self.renderer.white())
			if (l.length() < 500.0 and self.team != car.team and not car.is_demolished and car.physics.location.z + car.physics.velocity.z * 0.4 < 280.0):
				Enter_Flick(self, packet, Vec3(1, 0, 0))
				return
		
		# if (pos(my_car) - position).length() < 5500 and Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot((position - pos(my_car)).normal()) > 0.5:
			# Enter_Flick(self, packet, Vec3(1, 0, 0))
	
	

class KICKOFF(Enum):
	STRAIGHT = 1
	OFF_CENTER = 2
	DIAGONAL = 3

class Kickoff():
	
	def __init__(self, agent, packet):
		self.jumped = False
		self.wave_dashed = False
		self.kickoff_type = KICKOFF.STRAIGHT
		self.started_flip = False
		self.handbrake_timer = 10.0
		
		self.has_started = False
		
		my_car = packet.game_cars[agent.index]
		
		car_x = my_car.physics.location.x
		
		if abs(car_x) > 200:
			self.kickoff_type = KICKOFF.OFF_CENTER
		
		if abs(car_x) > 1500:
			self.kickoff_type = KICKOFF.DIAGONAL
		
		self.start_pos = pos(packet.game_cars[agent.index])
		
		self.timer = 0
		
	
	def update(self, agent, packet):
		
		if self.has_started:
			
			self.handbrake_timer += agent.delta
			
			my_car = packet.game_cars[agent.index]
			
			self.jumped = self.jumped or vel(my_car).z > 10
			
			my_goal = agent.field_info.my_goal
			goal_dir = my_goal.direction
			
			ball_pos_real = packet.game_ball.physics.location
			
			closest_car = None
			car_l = 10000
			for car in packet.game_cars:
				length = (car.physics.location - ball_pos_real).length()
				if car.team != agent.team and length < car_l:
					closest_car = car
			
			delta_hit = (calc_hit(closest_car, ball_pos_real).time if closest_car else 10) - calc_hit(my_car, ball_pos_real).time
			
			ball_pos = ball_pos_real - goal_dir * ((my_car.physics.location - ball_pos_real).length() * 0.25 if delta_hit > 0.4 else 200) # Steer goal - side
			car_pos = pos(my_car)
			
			car_to_ball = ball_pos - car_pos
			car_to_ball_real = ball_pos_real - car_pos
			local_car_to_ball = car_to_ball.align_from(my_car.physics.rotation)
			
			if car_to_ball.length() < vel(my_car).length() * 0.5:
				agent.controller_state = drive(agent, packet, ball_pos, 0.05)
			else:
				agent.controller_state = drive(agent, packet, ball_pos_real, 0.05)
			
			agent.controller_state.throttle = 1.0
			agent.controller_state.boost = (not self.jumped or vel(my_car).z > 10) and not self.wave_dashed
			
			agent.controller_state.jump = not self.jumped and vel(my_car).length() > (800 if self.kickoff_type == KICKOFF.DIAGONAL else 900)
			
			agent.controller_state.handbrake = self.handbrake_timer < 0.15
			if agent.controller_state.handbrake:
				agent.controller_state.steer = 0
			
			if not self.wave_dashed and self.jumped:
				self.handbrake_timer = 0.0
				if vel(my_car).z > 70:
					Align_Car_To(agent, packet, car_to_ball.normal() - Vec3(0, 0, 0.6), Vec3(0, 0, 1))
					agent.controller_state.boost = True
				else:
					Align_Car_To(agent, packet, car_to_ball.normal() + Vec3(0, 0, 0.7), Vec3(0, 0, 1))
					agent.controller_state.boost = pos(my_car).z > 60
					if pos(my_car).z < 40:
						self.wave_dashed = True
						agent.controller_state.jump = True
						agent.controller_state.yaw = 0.0
						agent.controller_state.roll = 0.0
						agent.controller_state.pitch = -1
			
			# Flips
			if (car_to_ball.length() < vel(my_car).length() * 0.4 and self.wave_dashed and delta_hit < 0.4) or self.started_flip:
				self.started_flip = True
				agent.controller_state.jump = vel(my_car).z < 10
				Flip_To_Ball(agent, packet, True)
			else:
				self.timer = 0.0
			
			# This is how we exit the maneuver
			return packet.game_ball.physics.location.x != 0
		
		else:
			
			agent.controller_state.throttle = 1.0
			agent.controller_state.boost = True
			
			self.has_started = (pos(packet.game_cars[agent.index]) - self.start_pos).length() > 100
			
			return False
		
	

class Aerial_Takeoff(Maneuver):
	def __init__(self, agent, packet, aerial_class):
		
		self.valid = True
		
		self.target_time = aerial_class.time
		self.offset = aerial_class.offset
		
		car = packet.game_cars[agent.index]
		location = Get_Ball_At_T(packet, agent.ball_prediction, self.target_time).physics.location
		
		car_face = Vec3(1, 0, 0).align_to(car.physics.rotation)
		car_up = Vec3(0, 0, 1).align_to(car.physics.rotation)
		
		# Calculate the appropriate takeoff type to minimize delta v
		
		target_dv = 800
		
		# MATH IS FUN
		takeoff_quick = Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300)
		takeoff_single = project_future(
			Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400
		)
		takeoff_fast_single = project_future(
			Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400 + car_face * 1000
		)
		takeoff_double = project_future(
			project_future(
				Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400
			),
			0.1
		)
		takeoff_double.velocity += car_up * 300
		
		takeoff_fast_double = project_future(
			project_future(
				Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400 + car_face * 1000
			),
			0.1, car_face * 1000
		)
		takeoff_fast_double.velocity += car_up * 300
		
		dv_quick = delta_v(takeoff_quick, location + self.offset, self.target_time, packet.game_info.world_gravity_z)
		dv_quick_len = dv_quick.length()
		
		dv_single = delta_v(takeoff_single, location + self.offset, self.target_time, packet.game_info.world_gravity_z)
		dv_s_len = dv_single.length()
		
		dv_fast_single = delta_v(takeoff_fast_single, location + self.offset, self.target_time, packet.game_info.world_gravity_z)
		dv_sf_len = dv_fast_single.length()
		
		dv_double = delta_v(takeoff_double, location + self.offset, self.target_time, packet.game_info.world_gravity_z)
		dv_d_len = dv_double.length()
		
		dv_fast_double = delta_v(takeoff_fast_double, location + self.offset, self.target_time, packet.game_info.world_gravity_z)
		dv_df_len = dv_fast_double.length()
		
		self.start_time = packet.game_info.seconds_elapsed
		
		# Should help with takeoff
		agent.controller_state.steer = 0
		
		# min_dv = min(dv_s_len, dv_sf_len,dv_d_len,dv_df_len,dv_quick_len)
		
		# Single tap jump button for one frame
		self.quick = False
		
		if dv_quick_len < target_dv and dv_quick.dot(car_face) > 0:
			self.approx_dv = dv_quick
			self.quick = True
			self.double_jump = False
			self.use_boost = False
			
		elif dv_s_len < target_dv and dv_single.dot(car_face) > 0:
			self.approx_dv = dv_single
			self.double_jump = False
			self.use_boost = True
			
		elif dv_sf_len < target_dv and dv_fast_single.dot(car_face) > 0:
			self.approx_dv = dv_fast_single
			self.double_jump = False
			self.use_boost = True
			
		elif dv_d_len < target_dv and dv_double.dot(car_face) > 0:
			self.approx_dv = dv_double
			self.double_jump = True
			self.use_boost = False
			
		elif dv_df_len < target_dv and dv_fast_double.dot(car_face) > 0:
			self.approx_dv = dv_fast_double
			self.double_jump = True
			self.use_boost = True
		else:
			self.valid = False
		
	
	def update(self, agent, packet):
		
		if not self.valid:
			return True
		
		# Align to aerial at the ball.
		Align_Car_To(agent, packet, self.approx_dv)
		agent.controller_state.steer = 0
		
		dt = packet.game_info.seconds_elapsed - self.start_time
		
		agent.controller_state.jump = dt < 0.2
		agent.controller_state.boost = self.use_boost
		
		if dt > (0.3 if self.double_jump else 0.2) or self.quick:
			if self.double_jump:
				agent.controller_state.jump = True
				agent.controller_state.pitch = 0
				agent.controller_state.yaw = 0
				agent.controller_state.roll = 0
			
			# Transfer into normal aerial
			agent.maneuver = Maneuver_Aerial(agent, packet, self.target_time, self.offset)
		
	

class Maneuver_Aerial(Maneuver):
	
	def __init__(self, agent, packet, time, offset = None):
		self.offset = Vec3() if offset is None else offset
		self.delta_t = time
		self.elapsed_time = 0
		# Calculate the delta v length to determine how often to boost. We want to keep this value relatively consistent.
		self.dv_l = self.get_dv(agent, packet).length()
	
	def update(self, agent, packet):
		self.elapsed_time += agent.delta
		self.delta_t -= agent.delta
		
		# Steer at the ball instead of the offset once we are close to hit.
		if self.delta_t < 0.5:
			self.offset = Vec3()
		
		my_car = packet.game_cars[agent.index]
		ball = Get_Ball_At_T(packet, agent.ball_prediction, self.delta_t)
		
		dv = self.get_dv(agent, packet)
		dot_vec = Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot(dv.normal())
		car_to_ball = ball.physics.location - my_car.physics.location
		
		Align_Car_To(agent, packet, dv) #, car_to_ball * -1 if dot_vec > 0.9 else Vec3())
		agent.controller_state.boost = dot_vec > (0.8 if self.delta_t < 0.2 else 0.9) and dv.length() > self.dv_l
		
		render_star(agent, ball.physics.location, agent.renderer.red())
		
		# Cancel aerial if the delta v goes to insane heights or if we have hit the location.
		return self.delta_t < 0 or (dv.length() > 2000 and self.delta_t > 0.5) or (my_car.has_wheel_contact and self.elapsed_time > 0.1)
	
	def get_dv(self, agent, packet):
		my_car = packet.game_cars[agent.index]
		ball = Get_Ball_At_T(packet, agent.ball_prediction, self.delta_t)
		return delta_v(my_car.physics, ball.physics.location + self.offset, self.delta_t, packet.game_info.world_gravity_z)
	








