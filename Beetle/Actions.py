from Utils import *

import math

from enum import Enum

class Maneuver:
	suspend_hit_prediction = True
	def update(self, agent, packet):
		pass

class Maneuver_Flip(Maneuver):
	def __init__(self, agent, packet, direction, time = 1):
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
			car = packet.game_cars[agent.index]
			return dt > self.max_time or car.has_wheel_contact
		return False


class Precise_Maneuver_Flip(Maneuver):
	def __init__(self, agent, packet, direction, time=1):
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
			car = packet.game_cars[agent.index]
			return dt > self.max_time or car.has_wheel_contact
		return False

class Maneuver_Half_Flip(Maneuver):
	def __init__(self, agent, packet):
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.f = Vec3(1, 0, 0).align_to(packet.game_cars[agent.index].physics.rotation)
		self.instant_abort = not packet.game_cars[agent.index].has_wheel_contact
	
	def update(self, agent, packet):
		if self.instant_abort:
			return True
		
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
	def __init__(self, agent, packet, time = 1):
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
			car = packet.game_cars[agent.index]
			return dt > self.max_time or car.has_wheel_contact
		return False

class Maneuver_Jump_Shot(Maneuver):
	def __init__(self, agent, packet, intersect_time, target):
		self.aim_dir = Vec3(1).align_to(packet.game_cars[agent.index].physics.rotation)
		self.start_time = packet.game_info.seconds_elapsed
		self.target = target
		self.delay = max(0.15, intersect_time - 0.05)
		if self.delay >= 0.3:
			self.jumpTimerMax = 0.2
			self.angleTimer = self.delay - 0.1
		else:
			self.angleTimer = 0
			self.jumpTimerMax = max(0.05, self.delay - 0.1)
		self.has_released = False
	
	def update(self,agent, packet):
		age = packet.game_info.seconds_elapsed - self.start_time
		controller_state = agent.controller_state
		controller_state.throttle = 0
		controller_state.boost = False
		car = packet.game_cars[agent.index]
		position = car.physics.location
		
		if age < self.angleTimer:
			# project_car = project_future(packet, car.physics, self.delay - age)
			# Align_Car_To(agent, packet,(self.target-project_car.location).normal(), Vec3(0, 0, 1))
			Align_Car_To(agent, packet, self.aim_dir, Vec3(0, 0, 1))
		
		if age < self.jumpTimerMax:
			controller_state.jump = True
		else:
			if age < self.delay - 0.05:
				controller_state.jump = False
			elif age < self.delay:
				vec = packet.game_ball.physics.location - car.physics.location + Vec3(20).align_to(car.physics.rotation)
				direction = vec.flatten().normal().align_from(car.physics.rotation)
				controller_state.jump = True
				controller_state.pitch = -direction.x
				controller_state.roll = direction.y
				controller_state.yaw = 0
			else:
				controller_state.jump = False
				# Wait an extra half second so that the recovery mode doesn't activate until the flip is finished.
				return age > self.jumpTimerMax + 1.2 or (car.has_wheel_contact and age > self.delay)
		return age > 0.1 and car.has_wheel_contact

# Needs to be worked on
class Maneuver_Push_Ball(Maneuver):
	def __init__(self, agent, packet):
		pass
	
	def update(self, agent, packet):
		
		my_car = packet.game_cars[agent.index]
		ball = self.next_bounce(agent, packet)
		
		dir = agent.field_info.opponent_goal.location - ball.physics.location
		
		# ref_vec = redirect_vect(ball.physics.velocity.flatten(), dir.flatten())
		
		# velocity_correct = (dir.normal(ball.physics.velocity.flatten().length()) - ball.physics.velocity.flatten()) * 0.25
		
		# if True: #velocity_correct.length() > 100:
			# velocity_correct = ref_vec * 100
		
		agent.controller_state = drive(agent, packet, ball.physics.location - dir.normal(130) + Vec3(-20).align_to(my_car.physics.rotation), (ball.game_seconds - packet.game_info.seconds_elapsed) * 0.98, allow_flips = False, avoid_walls = False)
		
		return (packet.game_ball.physics.location - my_car.physics.location).length() > 300
		
	def next_bounce(self, agent, packet):
		for slice in agent.ball_prediction.slices:
			if slice.physics.location.z < 120 and slice.game_seconds > packet.game_info.seconds_elapsed:
				return slice

class Maneuver_Flick(Maneuver):
	def __init__(self, packet, direction, time = 0.8):
		self.direction = direction
		self.start_time = packet.game_info.seconds_elapsed
		self.has_flipped = False
		self.max_time = time
	
	def update(self, agent, packet):
		dt = packet.game_info.seconds_elapsed - self.start_time
		if dt < 0.2:
			# Jump into air
			agent.controller_state.jump = True
			agent.controller_state.throttle = 0
			agent.controller_state.pitch = -1
		elif dt < 0.225:
			# Pause
			agent.controller_state.jump = False
			agent.controller_state.throttle = 0
		elif not self.has_flipped:
			# Perform the flip
			agent_car = packet.game_cars[agent.index]
			# self.direction = (agent_car.physics.location - packet.game_ball.physics.location).align_from(agent_car.physics.rotation)
			
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

class Air_Dribble(Maneuver):
	def __init__(self, agent, packet):
		ball_v = packet.game_ball.physics.velocity
		target_goal = agent.field_info.opponent_goal
		net_center = target_goal.location
		self.aim_pos = net_center + target_goal.direction * -50
		ball_peak_time = -ball_v.z / packet.game_info.world_gravity_z
		ball_peak_pos = pos(Get_Ball_At_T(packet, agent.ball_prediction, ball_peak_time))
		self.air_dribble_start = ball_peak_pos + ball_v.flatten()
		self.air_dribble_start.z = 500
		self.initial_touch_time = packet.game_info.seconds_elapsed
		
	
	def update(self, agent, packet):
		t_since_touch = packet.game_info.seconds_elapsed - self.initial_touch_time
		agent.controller_state.jump = t_since_touch < 0.15 # or (t_since_touch > 0.3 and t_since_touch < 0.35)
		
		my_car = packet.game_cars[agent.index]
		car_pos = pos(my_car) + Vec3(0, 0, 20).align_to(my_car.physics.rotation) #10
		
		# Air dribble stuff:
		# self.air_dribble_start
		# self.aim_pos
		
		aim_pos = self.aim_pos - Vec3(0, 0, 150)
		
		l = aim_pos - self.air_dribble_start
		l2 = (impulse_velocity(packet, packet.game_ball.physics, aim_pos, (aim_pos - pos(packet.game_ball)).length() * 0.0004) - vel(packet.game_ball))
		#l = aim_pos - pos(packet.game_ball)
		#l_n = l.normal()
		#aim_offset = Vec3(l_n.x * 45, l_n.y * 45, 80)
		agent.renderer.draw_string_3d(packet.game_ball.physics.location.UI_Vec3(), 2, 2, str((vel(packet.game_ball) * 0.01).length()), agent.renderer.red())
		aim_offset = (l2.normal(35) - vel(packet.game_ball) * 0.01).flatten(97) #45, 80; 20, 90
		
		# Get the time til the next touch
		next_touch_t = 0
		ball_pos = 0
		i = 0
		impulse = Vec3(0, 0, 0) #0
		while i < 6:
			ball = Get_Ball_At_T(packet, agent.ball_prediction, i)
			ball_pos = pos(ball) - aim_offset
			t = ((self.air_dribble_start - ball_pos).length() + (l.length() - (aim_pos - ball_pos).length())) / (l.length() * 2)
			if ball_pos.z + vel(ball).z * 0.1 < self.air_dribble_start.z + l.z * t and vel(ball).z < 0:
				next_touch_t = i
				impulse = impulse_velocity(packet, my_car.physics, ball_pos, i)
				break
			i += 0.1
		
		car_face = Vec3(1, 0, 0).align_to(my_car.physics.rotation)
		
		# Okay, so this needs to be fixed
		# aim_dir = pos(packet.game_ball) - car_pos - aim_offset
		
		aim_dir = impulse - vel(my_car)
		
		# aim_dir.z = max(aim_dir.z, 0) #is this really needed? Idk
		
		if t_since_touch < 1:
			Align_Car_To(agent, packet, aim_dir, Vec3(0, 0, 1))
		else:
			Align_Car_To(agent, packet, aim_dir, Vec3(0, 0, 0))
		
		#													ballRadius	 halfLength	 lengthOffset  jointToTop halfHeight
		#ball radius + car joint to front top edge middle: 	92.75 + sqrt((59.003689 + 13.87566)^2 + (38.83)^2)				= 175.328
		#ball radius + car joint to front side center: 		92.75 + sqrt((59.003689 + 13.87566)^2 + (38.83 - 18.079536)^2)	= 168.525862
		agent.controller_state.boost = car_face.dot(aim_dir.normal()) > 0.85 and (t_since_touch > 0.3) and aim_dir.length() > 180 #200
		
		render_star(agent, ball_pos, agent.renderer.purple())
		agent.renderer.draw_line_3d(self.air_dribble_start.UI_Vec3(), self.aim_pos.UI_Vec3(), agent.renderer.white())
		
		return (t_since_touch > 0.2 and (my_car.physics.location.z < 60 or my_car.has_wheel_contact))
	

# Todo: More intelligent handbrake usage
def drive(agent, packet, target_loc, time_allotted, target_v=-1, min_straight_spd=0, always_boost=False, allow_flips=True, avoid_walls = True, sensitive_boost = False):
	target_loc = target_loc.copy()
	
	car = packet.game_cars[agent.index]
	car_p = car.physics.location # Car position
	car_v = car.physics.velocity.flatten() # Car velocity
	car_dir = Vec3(1, 0, 0).align_to(car.physics.rotation) # Car direction
	
	if avoid_walls:
		target_loc.z = -car.physics.location.z * 2
	
	on_ground = abs(car_dir.z) < 0.1
	
	arc_turn = ArcTurn(car_p, car_dir, target_loc)
	if arc_turn.valid:
		arc_turn.render(agent.renderer, agent.renderer.blue())
	
	max_turn = turn_radius(car_v.length()) * 2
	
	if abs(car_p.x) < 885 and abs(car_p.y) > 5050 and abs(target_loc.x) > 800: # Car in net
		target_loc.x = 800 * sign(target_loc.x)
		time_allotted = 0
	
	car_to_loc_3d = target_loc - car_p
	car_to_loc = car_to_loc_3d.flatten() # - car_dir * 40
	
	# time_allotted = max(time_allotted, Time_to_Pos(car_to_loc.length(), car.physics.velocity.length(), car.boost).time * 0.75)
	
	path_length = calc_path_length(car, target_loc)
	target_speed = path_length / max(0.00001, time_allotted) #TODO: Handle divide by zero
	current_speed = car_v.length() # * sign(Vec3.dot(car_v, car_dir))
	
	#delta_speed = (target_speed - current_speed) * 0.01 - 0.1
	speed_err = current_speed - target_speed # >0 -> bot going too fast
	ang_vel = car.physics.angular_velocity.align_from(car.physics.rotation).z * 0.35
	ang_vel_correct = max(0, abs(ang_vel) - 0.5) * sign(ang_vel)
	heading_correct = correction(car, car_to_loc_3d)
	heading_err = heading_correct + ang_vel_correct
	heading_err_deg_abs = abs(math.degrees(heading_err))
	
	# Half flips
	if heading_err_deg_abs > 150 and allow_flips and car_to_loc.length() > 500 and car_v.length() < 200:
		agent.maneuver = Maneuver_Half_Flip(agent, packet)
		agent.maneuver_complete = False
		return MyControllerState()
	
	cs = MyControllerState()
	cs.steer = steer_for_heading_err(heading_err)
	# cs.handbrake = car_v.length() > 700 and (max_turn > arc_turn.radius or (car_to_loc.length() / car_v.length() > time_allotted and heading_err_deg_abs > 60)) and on_ground and car_v.dot(car_dir) > 0 and sign(ang_vel) != sign(heading_correct)
	cs.handbrake = car_v.length() > 700 and max_turn > arc_turn.radius and on_ground and car_v.dot(car_dir) > 0 and sign(ang_vel) != sign(heading_correct) # and car_to_loc.length() > 300 and heading_err_deg_abs > 20
	
	# if target_speed < min_straight_spd and heading_err_deg_abs < 10:
		# cs.throttle = 0.0
		# cs.boost = False
	# elif speed_err > 0:
		# if speed_err > 30:
			# cs.throttle = 0.0
		# else:
			# cs.throttle = 0.02
		# cs.boost = False
	# else: # <0
		# cs.throttle = 1 #constrain(-speed_err * 0.01 - 0.1)
		# should_boost = current_speed < 2275 and (always_boost or speed_err < (-500 / max(0.05, time_allotted)) or (current_speed > 1410 and speed_err < -1))
		# cs.boost = should_boost and not cs.handbrake
	
	agent.renderer.draw_string_3d(target_loc.flatten().UI_Vec3(), 2, 2, str(speed_err), agent.renderer.red())
	if abs(speed_err + 10) < 20:
		cs.throttle = 0.2
	elif speed_err > 0 and speed_err < 200:
		cs.throttle = 0
	else:
		cs.throttle = -speed_err * 0.01 + 0.03
		# cs.throttle = 1
	
	should_boost = current_speed < 2275 and (always_boost or speed_err < -200 or (sensitive_boost and speed_err < -100))
	cs.boost = should_boost and not cs.handbrake
	
	dist_until_brake = car_to_loc.length() - brake_dist(car_v.length(), target_v) - (2.0 * car_v.length() * (1.0 / 60.0))
	if target_v != -1 and car_v.length() > target_v - 10 and dist_until_brake < 0: # brake
		cs.throttle = -1
		cs.boost = False
	
	if speed_err < -500 and car_v.length() > 1000 and car_to_loc.length() > (car_v.length() + 500) * (1.5 if target_v == -1 else 3) and allow_flips and Vec3(1, 0, 0).align_to(car.physics.rotation).dot(car_to_loc.normal()) > 0.95 and car.boost < 15 and Vec3(1, 0, 0).align_to(car.physics.rotation).dot((car_to_loc_3d).normal()) > 0.95:
		Enter_Flip(agent, packet, Vec3(1, 0, 0), low_flip = True)
	
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
	should_boost = current_speed < 2300 and (speed_err < -400 or (current_speed > 1410 and speed_err < -1))
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
	if speed_err < -1000 and car_v.length() > 1000 and car_to_loc.length() > car_v.length() * 1.5 and allow_flips and Vec3(1, 0, 0).align_to(car.physics.rotation).dot(car_to_loc.normal()) > 0.95 and (car_v.length() < 1500 or car.boost < 10):
		Enter_Flip(agent, packet, Vec3(1, 0, 0))
	
	return cs


# Drives Beetle on an line-arc-line path
class Line_Arc_Line_Driver(Maneuver):
	
	def __init__(self, agent, packet, line_arc_line, do_flip = True, execute_time = -1):
		self.suspend_hit_prediction = False
		self.line_arc_line = line_arc_line
		if execute_time < 0:
			self.execute_time = line_arc_line.calc_time(packet.game_cars[agent.index])
		else:
			self.execute_time = execute_time
		self.stage = 0
		self.p_time = packet.game_info.seconds_elapsed
		self.p_car_v = 1000
		self.do_flip = do_flip
		
		# Used to make sure ball doesn't move
		self.predicted_ball_loc = Get_Ball_At_T(packet, agent.ball_prediction, self.execute_time).physics.location
	
	def update(self, agent, packet):
		
		render_star(agent, self.predicted_ball_loc, agent.renderer.yellow())
		
		delta = packet.game_info.seconds_elapsed - self.p_time
		self.p_time = packet.game_info.seconds_elapsed
		
		self.execute_time -= delta
		
		if self.execute_time < 0:
			return True
		
		if self.line_arc_line.valid:
			self.line_arc_line.render(agent)
		else:
			return True
		
		agent.controller_state.handbrake = False
		
		if self.stage == 1:
			
			my_car = packet.game_cars[agent.index]
			
			car_v = my_car.physics.velocity.length()
			
			a = (car_v - self.p_car_v) * delta
			
			phys = project_future(packet, my_car.physics, delta * 2)
			
			vector = Vec2.cast(phys.location) - self.line_arc_line.arc_center
			vector2 = Vec2.cast(my_car.physics.location) - self.line_arc_line.arc_center
			
			# off = (vector.length() - self.line_arc_line.arc_radius) * 0.05
			
			ang = vector.inflate().angle_between(self.line_arc_line.a2.inflate())
			ang2 = vector2.inflate().angle_between(self.line_arc_line.a2.inflate())
			if self.line_arc_line.offset.dot(vector) < 0:
				ang = math.pi * 2 - ang
				ang2 = math.pi * 2 - ang2
			
			arc_dir = self.line_arc_line.arc_dir
			
			if sign(ang) != sign(arc_dir):
				ang *= -1
			
			# self.line_arc_line.arc_dir
			
			a = self.line_arc_line.a2.angle() + ang - arc_dir * 0.4
			p = self.line_arc_line.arc_center + Vec2(math.cos(a) * self.line_arc_line.arc_radius, math.sin(a) * self.line_arc_line.arc_radius)
			car_to_loc_3d = p.inflate() - phys.location
			
			heading_err = correction(my_car, car_to_loc_3d)
			agent.controller_state.steer = -heading_err * abs(heading_err) * 20
			
			agent.renderer.draw_string_3d((my_car.physics.location + Vec3(0, 0, 1000)).UI_Vec3(), 2, 2, str(int(math.degrees(ang))), agent.renderer.yellow())
			
			# s_mag = 1 / (self.line_arc_line.arc_radius * curvature(car_v)) + constrain(off) * 0.5
			# agent.controller_state.steer = constrain(s_mag * -sign(correction(my_car, self.line_arc_line.p2.inflate() - my_car.physics.location)))
			
			render_star(agent, p.inflate(20), agent.renderer.blue(), 50)
			
			target_v = (ang2 * self.line_arc_line.arc_radius + self.line_arc_line.offset.length()) / self.execute_time
			
			agent.controller_state.throttle = (target_v - car_v) * 0.03
			if abs(agent.controller_state.throttle) < 0.2:
				agent.controller_state.throttle = 0.1
			
			agent.controller_state.boost = car_v + 100 < target_v and target_v > 700
			
			# Transition into final part once we are facing the right direction
			if (self.line_arc_line.offset).normal(-1).dot(Vec3(1, 0, 0).align_to(my_car.physics.rotation)) > 0.99:
				self.stage += 1
			
			self.p_car_v = car_v
			
		else:
			p = self.line_arc_line.p1 if self.stage == 0 else self.line_arc_line.target
			
			my_car = packet.game_cars[agent.index]
			car_to_loc_3d = p.inflate() - my_car.physics.location
			
			car_v = my_car.physics.velocity.length()
			
			if self.stage == 0:
				target_v = (car_to_loc_3d.length() + self.line_arc_line.arc_length + self.line_arc_line.offset.length()) / self.execute_time
			else:
				target_v = car_to_loc_3d.length() / self.execute_time
			
			heading_err = correction(my_car, car_to_loc_3d)
			
			agent.controller_state.steer = steer_for_heading_err(heading_err)
			agent.controller_state.throttle = (target_v - car_v + 50) * 0.02
			if abs(agent.controller_state.throttle) < 0.2:
				agent.controller_state.throttle = 0.1
			
			agent.controller_state.boost = car_v + 100 < target_v and target_v > 700
			
			agent.controller_state.handbrake = abs(heading_err) > math.pi * 0.5 and car_v > 500 and abs(my_car.physics.angular_velocity.z) < 5
			
			if car_to_loc_3d.length() < car_v * delta * 2 + 50:
				self.stage += 1
			
			self.p_car_v = car_v
		
		# Check to make sure the shot is still valid
		ball = Get_Ball_At_T(packet, agent.ball_prediction, self.execute_time).physics
		
		# Path needs to be abandoned, ball has moved
		if (ball.location - self.predicted_ball_loc).length() > 50:
			return True
		
		# Jump shot stuff
		ball_offset = 93
		angle = abs(math.degrees(my_car.physics.rotation.angle_to_vec(ball.location.flatten())))
		car_offset = agent.hitbox.get_offset_by_angle(angle)
		total_offset = car_offset+ball_offset
		
		targetDistance = abs((my_car.physics.location- ball.location).length())
		speed = clamp(abs(my_car.physics.velocity.length()),0.001,2300)
		
		up_v = Vec3(0, 0, 1).align_to(my_car.physics.rotation)
		
		# Project car into the future
		sub_step = project_future(packet, Psuedo_Physics(location=my_car.physics.location,velocity=my_car.physics.velocity+up_v*300), min(self.execute_time, 0.2), up_v * 1400)
		future_car = project_future(packet, sub_step, max(0, self.execute_time - 0.2))
		
		render_star(agent, future_car.location, agent.renderer.purple())
		
		push_ball = ball.location.z < 100 and ball.velocity.length() < 1000 and my_car.physics.velocity.length() < 1000
		
		if push_ball:
			if self.execute_time < 0.2:
				agent.maneuver = Maneuver_Push_Ball(agent, packet)
		else:
			# if speed * self.execute_time < clamp(targetDistance - total_offset,0,99999) and self.execute_time < 1 and self.stage == 2:
			if future_car.location.z < ball.location.z and self.execute_time < 0.9 and self.stage == 2 and self.do_flip:
				agent.maneuver = Maneuver_Jump_Shot(agent, packet, self.execute_time, ball.location)
			
		
		return False
	

def Dribble(self, packet, position: Vec3):
	
	position_2 = position.copy()
	position_2.z = 100
	
	prediction = self.ball_prediction
	
	my_car = packet.game_cars[self.index]
	
	car_pos = my_car.physics.location + Vec3(30).align_to(my_car.physics.rotation)
	
	car_direction = Vec3(1, 0, 0).align_to(my_car.physics.rotation)
	
	ball_vel = packet.game_ball.physics.velocity
	ball_pos = packet.game_ball.physics.location + ball_vel * 0.17
	
	ball_predict = Get_Ball_At_T(packet, prediction, 0.5).physics
	
	# if Vec3(car_pos.x - ball_pos.x, car_pos.y - ball_pos.y, 0).flatten().length() > 500 or ball_pos.z - car_pos.z > 400:
		# ball_pos = ball_predict.location
	
	car_to_p2 = (position_2 - car_pos).normal(1200)
	dir_vec = (car_to_p2 - ball_vel) * 0.03
	
	angle = clamp(correction(my_car, dir_vec.normal()) * 2, -math.pi * 0.7, math.pi * 0.7)
	
	dir = Vec3(car_direction.y * math.sin(-angle) * 2 - car_direction.x * math.cos(-angle), -car_direction.y * math.cos(-angle) * 2 - car_direction.x * math.sin(-angle), 0.0)
	
	multiplier = 0.5
	
	if Vec3(car_pos.x - ball_pos.x, car_pos.y - ball_pos.y, 0.0).length() < 250.0 and abs(ball_vel.z) < 300.0 and ball_predict.location.z < 200.0:
		multiplier = 1
	
	position = ball_pos + dir * multiplier * min(27, dir_vec.length())
	
	position.z *= 0.2
	
	car_vel = my_car.physics.velocity * 0.15
	
	car_to_pos = position - car_pos - car_vel
	
	steer_correction_radians = correction(my_car, car_to_pos)
	
	self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians))
	
	if car_to_pos.flatten().dot(car_direction) > 0.0 or car_to_pos.length() > 250:
		self.controller_state.throttle = constrain(car_to_pos.length() / 20 + 0.2)
	else:
		self.controller_state.throttle = -constrain(car_to_pos.length() / 20 + 0.2)
		self.controller_state.steer = - self.controller_state.steer
	
	if abs(self.controller_state.throttle) < 0.1:
		self.controller_state.throttle = 0.02
	elif self.controller_state.throttle < 0 and self.controller_state.throttle > -0.2:
		self.controller_state.throttle = 0
	
	if car_to_pos.length() > 300.0:
		self.controller_state.boost = abs(steer_correction_radians) < 0.2 and (car_to_pos - car_vel * 10).length() > 200 and car_to_pos.dot(car_to_pos - car_vel * 3) > 0.0
	else:
		l = car_to_pos.flatten().length()
		self.controller_state.boost = l > 30 and ball_pos.z < 200 and abs(steer_correction_radians) < math.pi * 0.25
	
	self.controller_state.jump = False
	self.controller_state.pitch = 0.0
	
	self.controller_state.handbrake = car_to_pos.length() > 100 and abs(steer_correction_radians) > math.pi * 0.5
	

# The class that controls Beetle's kickoffs
class KICKOFF(Enum):
	STRAIGHT = 1
	OFF_CENTER = 2
	DIAGONAL = 3

# Special side flip for kickoffs
class Kickoff_Flip(Maneuver):
	def __init__(self, agent, packet):
		self.start = packet.game_info.seconds_elapsed
	
	def update(self, agent, packet):
		delta = packet.game_info.seconds_elapsed - self.start
		agent.controller_state.boost = delta < 0.2
		if delta < 0.1:
			agent.controller_state.jump = True
		elif delta > 0.2 and delta < 0.3:
			my_car = packet.game_cars[agent.index]
			vec = packet.game_ball.physics.location - my_car.physics.location
			vec = vec + packet.game_ball.physics.velocity.normal(vec.length() * 0.5)
			direction = vec.align_from(my_car.physics.rotation).flatten()
			agent.controller_state.jump = True
			agent.controller_state.roll = sign(direction.y)
		else:
			agent.controller_state.jump = False
			
			if delta > 0.3:
				my_car = packet.game_cars[agent.index]
				Align_Car_To(agent, packet, my_car.physics.velocity.flatten(), Vec3(0, 0, 1))
			
			return delta > 0.7

class Stall_Flip(Maneuver):
	def __init__(self, agent, packet):
		self.start = packet.game_info.seconds_elapsed
		self.flipped = False
	
	def update(self, agent, packet):
		delta = packet.game_info.seconds_elapsed - self.start
		if delta < 0.1:
			agent.controller_state.jump = True
		elif delta > 0.15 and not self.flipped:
			my_car = packet.game_cars[agent.index]
			vec = packet.game_ball.physics.location - my_car.physics.location
			vec = vec + packet.game_ball.physics.velocity.normal(vec.length() * 0.5)
			direction = vec.align_from(my_car.physics.rotation).flatten()
			agent.controller_state.jump = True
			agent.controller_state.roll = sign(direction.y) * 0.6
			agent.controller_state.pitch = -1
			self.flipped = True
		else:
			my_car = packet.game_cars[agent.index]
			agent.controller_state.jump = False
			# agent.controller_state.pitch = 1
			# agent.controller_state.roll = 0
			
			# if delta > 0.7:
				# my_car = packet.game_cars[agent.index]
				# Align_Car_To(agent, packet, my_car.physics.velocity.flatten(), Vec3(0, 0, 1))
			
			if delta > 0.7:
				Align_Car_To(agent, packet, my_car.physics.velocity.flatten(), Vec3(0, 0, 1))
			else:
				Align_Car_To(agent, packet, my_car.physics.velocity.flatten())
				agent.controller_state.pitch = 1
			
			return delta > 1
		return False

class Kickoff(Maneuver):
	
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
		
		self.kickoff_dir = sign(car_x)
		
		self.kickoff_move = None
		
	
	def update(self, agent, packet):
		
		
		if not packet.game_info.is_round_active:
			self.__init__(agent, packet)
			agent.controller_state.throttle = 1
			agent.controller_state.boost = True
			
			return False
		
		if self.has_started:
			
			self.handbrake_timer += agent.delta
			
			my_car = packet.game_cars[agent.index]
			
			my_goal = agent.field_info.my_goal
			goal_dir = my_goal.direction
			
			ball_pos_real = packet.game_ball.physics.location
			
			closest_car = None
			car_l = 10000
			for car in packet.game_cars:
				length = (car.physics.location - ball_pos_real).length()
				if car.team != agent.team and length < car_l:
					closest_car = car
			
			my_hit_time = calc_hit(my_car, ball_pos_real).time
			delta_hit = (calc_hit(closest_car, ball_pos_real).time if closest_car else 10) - my_hit_time
			
			# Ensure we deflect kickoff to the side
			if self.kickoff_type == KICKOFF.DIAGONAL:
				offset = Vec3()
			else:
				offset = Vec3(self.kickoff_dir * 40, 0, 0)
			
			ball_pos_real += offset
			
			ball_pos = ball_pos_real + offset - goal_dir * (130 if my_hit_time < 0.4 else 200)
			
			car_pos = pos(my_car)
			
			car_to_ball = ball_pos - car_pos
			car_to_ball_real = ball_pos_real - car_pos
			local_car_to_ball = car_to_ball.align_from(my_car.physics.rotation)
			
			if self.kickoff_type == KICKOFF.OFF_CENTER and not self.wave_dashed:
				off_center_pos = Vec3(sign(car_pos.x) * 55, 2900 * sign(car_pos.y), 0)
				
				# Stall flip
				if not self.jumped:
					if (car_pos - off_center_pos).length() < 600:
						self.kickoff_move = Stall_Flip(agent, packet)
						self.jumped = True
					
					agent.controller_state = drive(agent, packet, off_center_pos, 0.01, allow_flips = False)
				else:
					self.wave_dashed = self.kickoff_move.update(agent, packet)
				
				agent.controller_state.throttle = 1
				agent.controller_state.boost = True
			else:
				# Wave dash
				self.jumped = self.jumped or vel(my_car).z > 10
				
				if car_to_ball.length() < vel(my_car).length() * 0.5:
					agent.controller_state = drive(agent, packet, ball_pos, 0.01, allow_flips = False)
				else:
					agent.controller_state = drive(agent, packet, ball_pos, 0.01, allow_flips = False)
				
				agent.controller_state.jump = not self.jumped and vel(my_car).length() > 900
				
				agent.controller_state.handbrake = self.handbrake_timer < 0.15
				if agent.controller_state.handbrake:
					agent.controller_state.steer = 0
				
				agent.controller_state.boost = True
				agent.controller_state.throttle = 1.0
				
				if not self.wave_dashed and self.jumped:
					self.handbrake_timer = 0.0
					if vel(my_car).z > 90:
						Align_Car_To(agent, packet, car_to_ball.normal() - Vec3(0, 0, 0.5), Vec3(0, 0, 1))
						# agent.controller_state.boost = True
					else:
						Align_Car_To(agent, packet, car_to_ball.normal() + Vec3(0, 0, 0.7), Vec3(0, 0, 1))
						if pos(my_car).z < 45:
							self.wave_dashed = True
							agent.controller_state.jump = True
							agent.controller_state.yaw = 0.0
							agent.controller_state.roll = 0.0
							agent.controller_state.pitch = -1
			
			# Flips
			if ((car_to_ball.length() - 150) < vel(my_car).length() * (0.25 if self.kickoff_type == KICKOFF.DIAGONAL else 0.22) and self.wave_dashed and delta_hit < 0.2) or self.started_flip:
				self.started_flip = True
				agent.maneuver = Kickoff_Flip(agent, packet)
				agent.maneuver_complete = False
			else:
				self.timer = 0.0
			
			# This is how we exit the maneuver
			return not packet.game_info.is_kickoff_pause
		
		else:
			
			agent.controller_state.throttle = 1.0
			agent.controller_state.boost = True
			
			self.has_started = (pos(packet.game_cars[agent.index]) - self.start_pos).length() > 10
			
			return False
		
	

def get_jump_time(z, packet):
	
	# Constant time for objects that are low enough. (May need tweaks)
	if z < 130:
		return 0
	
	g = packet.game_info.world_gravity_z
	a = 1400
	start_height = 17
	amount_below = 20 # Might be able to get away with increasing this.
	c = 0.5 * (a + g) * (0.2 * 0.2) + 300 * 0.2
	a2 = (a + g) * 0.2 + 300
	
	# Solve for t in this equation:
	# 0.5(-g)(t - 0.2) ^ 2 + a2(t - 0.2) + c + start_height = z
	#
	# 0.5(-g)(t - 0.2) ^ 2 + a2(t - 0.2) + c + start_height - z = 0
	
	_a = 0.5 * g
	_b = a2
	_c = c + start_height - z # + amount_below
	
	discriminant = _b * _b - 4 * _a * _c
	
	if discriminant < 0:
		return 0.88
	
	t = (-_b - math.sqrt(discriminant)) / (2 * a)
	
	return max(0.2, t + 0.2)
	

# Todo: Re-vamp this
def JumpShot_Handler(agent,packet,ideal_time = 0, perfect_world = False, cautious = True):
	
	hit = agent.hit
	
	rolling = abs(packet.game_ball.physics.velocity.z) < 1000 and packet.game_ball.physics.location.z < 300
	
	enemyGoal = agent.field_info.opponent_goal.location - agent.field_info.opponent_goal.direction * 93
	myGoal = agent.field_info.my_goal.closest_point(packet.game_ball.physics.location) - agent.field_info.my_goal.direction * 93
	
	my_car = packet.game_cars[agent.index]
	
	touch = agent.hit_package.flip_touch.copy()
	
	# touch.time = max(ideal_time + 0.25, touch.time - 0.1)
	
	rotator = my_car.physics.rotation
	car_location = my_car.physics.location
	grounded,on_wall = grounded_and_wall_check(agent,packet)
	shot_limit = 0.9
	
	ball_offset = 93
	offset_mul = 1 if perfect_world else 1
	
	if (touch.location- myGoal).length() < 2500:
		direction = (touch.location - myGoal).flatten().normal()
	else:
		direction = (enemyGoal - touch.location).flatten().normal()
	
	if rolling and (touch.location - my_car.physics.location).normal().dot(direction) < 0 and cautious:
		direction = Vec3(sign(touch.location.x), agent.field_info.my_goal.direction.y * 0.25).normal()
	
	if rolling and (touch.location - my_car.physics.location).normal().dot(direction) < -0.7 and cautious:
		direction = Vec3(sign(touch.location.x - car_location.x), 0.25).normal()
	
	# + Vec3(20).align_to(car.physics.rotation) 
	# Refine the touch
	car_vel_len = my_car.physics.velocity.length()
	found_touch = False
	i = max(0, touch.time - 0.2)
	jump_time = None
	step = (1/60)
	while i < touch.time + 0.3:
		ball_loc = hit.ball_at_t(agent, packet, agent.ball_prediction, i, max_height = 265) - Vec3(20).align_to(my_car.physics.rotation)
		angle = abs(math.degrees(rotator.angle_to_vec(ball_loc.flatten())))
		total_offset = agent.hitbox.get_offset_by_angle(angle) * offset_mul
		
		loc = ball_loc - (direction * total_offset)
		
		jump_time = get_jump_time(loc.z, packet) + 0.05
		turn_time = 0 # calc_turn(my_car, loc)
		if jump_time + turn_time - step > i or ball_loc.z > 265:
			i += step
			continue
		
		vel_achievable = Vel_at_Time(car_vel_len, i - jump_time - turn_time, my_car.boost)
		
		jump_offset = vel_achievable * jump_time
		
		length = max(0, (car_location - loc).flatten().length() - jump_offset - 20)
		
		t = Time_to_Pos(length, car_vel_len, my_car.boost)
		
		# t = calc_hit(my_car, , minimum=True, angle_correct=False)
		
		if t.time < i - jump_time - turn_time + step:
			touch.time = i
			touch.location = ball_loc
			found_touch = True
			break
		
		i += step
	
	targetDistance = abs((car_location- touch.location).length())
	angle = abs(math.degrees(rotator.angle_to_vec(touch.location.flatten())))
	car_offset = agent.hitbox.get_offset_by_angle(angle)
	total_offset = (car_offset + ball_offset) * offset_mul
	
	ideal_position = touch.location - (direction * total_offset)
	
	bad_position = touch.location + (direction * total_offset)
	
	render_star(agent, touch.location, agent.renderer.yellow(), 20)
	render_star(agent, ideal_position, agent.renderer.blue(), 20)
	
	targetDistance = abs((car_location- touch.location).length())
	agent.controller_state = drive(agent, packet, ideal_position.flatten(), min(2, touch.time), allow_flips=agent.hit.hit_time - touch.time < 0.2)
	
	# push_ball = touch.location.z < 120 and packet.game_ball.physics.velocity.length() < 1000 and my_car.physics.velocity.length() < 1000
	
	# if push_ball and (not rolling or (touch.location - my_car.physics.location).normal().dot(touch.location - myGoal) > 0):
		# if touch.time < 0.3:
			# agent.maneuver = Maneuver_Push_Ball(agent, packet)
			# agent.maneuver_complete = False
	
	# elif grounded and not on_wall:
	if grounded and not on_wall:
		j_t = max(0.15, jump_time)
		if (agent.hit.hit_time - touch.time > -0.15 or (packet.game_cars[agent.hit.hit_index].has_dribble and agent.hit.hit_time - touch.time > -0.3)) and touch.time - j_t < 0.05 and (my_car.physics.location + my_car.physics.velocity * j_t - touch.location).flatten().length() < 250:
			agent.maneuver = Maneuver_Jump_Shot(agent, packet, touch.time, touch.location)
			agent.maneuver_complete = False
	

class Aerial_Takeoff(Maneuver):
	def __init__(self, agent, packet, aerial_class):
		
		self.suspend_hit_prediction = False
		
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
		takeoff_single = project_future(packet, 
			Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400
		)
		takeoff_fast_single = project_future(packet, 
			Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400 + car_face * 1000
		)
		takeoff_double = project_future(packet, 
			project_future(packet, 
				Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400
			),
			0.05
		)
		takeoff_double.velocity += car_up * 300
		
		takeoff_fast_double = project_future(packet, 
			project_future(packet, 
				Psuedo_Physics(car.physics.location, car.physics.velocity + car_up * 300), 0.2, car_up * 1400 + car_face * 1000
			),
			0.05, car_face * 1000
		)
		takeoff_fast_double.velocity += car_up * 300
		
		dv_quick = delta_v(takeoff_quick, location + self.offset, self.target_time, packet.game_info.world_gravity_z)
		dv_quick_len = dv_quick.length()
		
		dv_single = delta_v(takeoff_single, location + self.offset, self.target_time - 0.2, packet.game_info.world_gravity_z)
		dv_s_len = dv_single.length()
		
		dv_fast_single = delta_v(takeoff_fast_single, location + self.offset, self.target_time - 0.2, packet.game_info.world_gravity_z)
		dv_sf_len = dv_fast_single.length()
		
		dv_double = delta_v(takeoff_double, location + self.offset, self.target_time - 0.25, packet.game_info.world_gravity_z)
		dv_d_len = dv_double.length()
		
		dv_fast_double = delta_v(takeoff_fast_double, location + self.offset, self.target_time - 0.25, packet.game_info.world_gravity_z)
		dv_df_len = dv_fast_double.length()
		
		self.start_time = packet.game_info.seconds_elapsed
		
		# Should help with takeoff
		agent.controller_state.steer = 0
		
		# min_dv = min(dv_s_len, dv_sf_len,dv_d_len,dv_df_len,dv_quick_len)
		
		# Single tap jump button for one frame
		self.quick = False
		
		if dv_quick_len < target_dv * 0.7 and dv_quick.normal().dot(car_face) > 0.5:
			self.approx_dv = dv_quick
			self.quick = True
			self.double_jump = False
			self.use_boost = False
			
		elif dv_s_len < target_dv * 0.8 and dv_single.normal().dot(car_face) > 0.5:
			self.approx_dv = dv_single
			self.double_jump = False
			self.use_boost = True
			
		elif dv_sf_len < target_dv * 0.9 and dv_fast_single.normal().dot(car_face) > 0.5:
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
		
		if dt > (0.25 if self.double_jump else 0.2) or self.quick:
			if self.double_jump:
				agent.controller_state.jump = True
				agent.controller_state.pitch = 0
				agent.controller_state.yaw = 0
				agent.controller_state.roll = 0
			
			# Transfer into normal aerial
			agent.maneuver = Maneuver_Aerial(agent, packet, self.target_time - dt, self.offset)
		
	

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
		return delta_v(my_car.physics, ball.physics.location + self.offset * (0.75 if self.delta_t < 0.5 else 1), self.delta_t, packet.game_info.world_gravity_z)
	







