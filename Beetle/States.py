
from Structs import Vec3
from Utils import *
from Actions import *
from math import degrees

# A state either returns None (for allowing the program to select state) or a state which will be run on the next get_output
# Can also return itself to lock its own state

class State:
	def output(self, agent, packet):
		raise NotImplementedError

class Recovery(State):
	def __init__(self, prev_state):
		self.prev_state = prev_state
	def output(self, agent, packet):
		Align_Car_To(agent, packet, packet.game_cars[agent.index].physics.velocity.flatten(), Vec3(0, 0, 1))
		agent.controller_state.throttle = 1
		# Experimental
		# touch = agent.touch
		# if agent.touch_type == TouchType.aerial and touch.is_garunteed and not packet.game_cars[agent.index].has_wheel_contact:
			# agent.maneuver_complete = False
			# agent.maneuver = Maneuver_Aerial(agent, packet, touch.time + 0.2)
			# agent, packet, time, offset = None
		
		return self.prev_state if packet.game_cars[agent.index].has_wheel_contact else self

class Carry_Ball(State):
	def output(self, agent, packet):
		Dribble(agent, packet, agent.field_info.opponent_goal.location)
		my_car = packet.game_cars[agent.index]
		if agent.packet.game_ball.physics.location.z < 120 or (agent.packet.game_ball.physics.location - my_car.physics.location).length() > 200 or not my_car.has_wheel_contact:
			return Defend()

class Jumpshot_Handler(State):
	def output(self,agent,packet,perfect_world = False):
		enemyGoal = Vec3(0, 5200 * -sign(agent.team), 100)
		myGoal = Vec3(0, 5200 * sign(agent.team), 100)
		touch = agent.touch
		my_car = packet.game_cars[agent.index]
		rotator = my_car.physics.rotation
		car_location = my_car.physics.location
		grounded,on_wall = grounded_and_wall_check(agent,packet)
		targetDistance = abs((car_location- touch.location).length())
		shot_limit = 1
		
		ball_offset = 93
		angle = abs(math.degrees(rotator.angle_to_vec(touch.location.flatten())))
		car_offset = agent.hitbox.get_offset_by_angle(angle)
		total_offset = car_offset+ball_offset
		
		
		if not perfect_world:
			total_offset*=.75
		
		if abs((car_location- myGoal).length()) < 2500:
			direction = (touch.location.flatten() - myGoal.flatten()).normal()
			ideal_position = touch.location-(direction*total_offset)
		
		else:
			direction = (enemyGoal.flatten() - touch.location.flatten()).normal()
			ideal_position = touch.location - (direction * total_offset)
		
		bad_position = touch.location + (direction * total_offset)
		
		targetDistance = abs((car_location- touch.location).length())
		agent.controller_state = drive(agent, packet, ideal_position.flatten(),touch.time)
		if grounded and not on_wall:
			futurePos = car_location + my_car.physics.velocity * agent.delta
			if touch.time < shot_limit:
				if abs((futurePos-ideal_position).length()) < abs((futurePos-bad_position).length()):
					speed = clamp(abs(my_car.physics.velocity.length()),0.001,2300)
					if speed * touch.time >= clamp(targetDistance -total_offset,0,99999):
						agent.maneuver = Maneuver_Jump_Shot(agent, packet, touch.time, touch.location)
						agent.maneuver_complete = False
						return Defend()
		



class Defend(State):
	def __init__(self):
		self.carry_timer = 0
	
	def output(self, agent, packet):
		
		touch = agent.touch
		hit = agent.hit
		
		bTOGT = ball_in_my_goal_time(agent, packet)
		
		render_star(agent, ball_loc_at_time(agent, packet, bTOGT), agent.renderer.red())
		
		ball_intercept_time = ball_past_y_time(agent, packet, 4850) #4970
		render_star(agent, touch.location, agent.renderer.yellow())
		
		dest = touch.location.flatten()
		
		my_car = packet.game_cars[agent.index]
		ball = packet.game_ball.physics
		
		my_goal = agent.field_info.my_goal
		
		if agent.field_info.my_goal.direction.dot((packet.game_ball.physics.location - my_car.physics.location).normal()) < -0.1 and (abs(dest.x) < 1200 or dest.y * my_goal.direction.y > -4000):
			dest += Vec3(200 * sign(my_car.physics.location.x - packet.game_ball.physics.location.x), -agent.field_info.my_goal.direction.y * 500, 0)
			agent.controller_state = drive(agent, packet, dest, touch.time - 0.2, allow_flips=True)
		else:
			agent.controller_state = drive(agent, packet, dest, touch.time, allow_flips=True)
		# tried to shoehorn jumpshot in here, failed epicly
		# if agent.touch.location.z > 120 and agent.touch.location.z <= 265:
		# 	return Jumpshot_Handler().output(agent,packet,perfect_world = False)
		
		# if (bTOGT != -1 or (ball.location + ball.velocity.flatten() - my_goal.location).length() < 1000 or agent.touch_type != TouchType.ground) and touch.time < 0.4:
			# Flip_To_Ball(agent, packet)
		
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		c_b_o = (my_car.physics.location - touch.location).flatten()
		
		d_p_v = b_g_o.angle_between(c_b_o)
		d_p_v_2 = b_g_o.angle_between(agent.field_info.opponent_goal.direction)
		
		b_g_o_len = b_g_o.length()
		
		b_g_len = (touch.location - my_goal.location).flatten().length()
		
		c_boost = get_corner_boost_index(agent)
		
		pad_grab_time = calc_hit(my_car, agent.field_info.full_boosts[c_boost].location).time + Time_to_Pos((agent.field_info.full_boosts[c_boost].location - my_goal.location).length(), 0, 100).time if c_boost >= 0 else 10000
		
		# We keep our own dribble timer in order to be more restrictive than the dribble tracker.
		if abs(160 - ball.location.z) < 30 and (ball.location - my_car.physics.location).length() < 200:
			self.carry_timer += agent.delta
			if self.carry_timer > 1:
				return Carry_Ball()
		elif agent.touch_type == TouchType.flip and touch.time < hit.hit_time and touch.time < 2:
			return Jumpshot_Handler()
		elif any((agent.team != car.team and car.has_dribble) for car in packet.game_cars):
			self.carry_timer = 0
		elif (d_p_v < math.pi * 0.4 and d_p_v_2 < math.pi * 0.3 and touch.time < hit.hit_time - 0.5 and b_g_o_len < 6000):
			return Take_Shot()
		# For testing puposes only for now. Want to test aerials.
		elif agent.touch_type == TouchType.aerial and touch.is_garunteed and touch.time > 1 and touch.location.z > 300:
			if my_goal.direction.dot(my_car.physics.location - touch.location) > 0.0:
				return Align_For_Aerial(agent, packet, touch.time, my_goal.direction * -90 + Vec3(sign(my_car.physics.location.x - touch.location.x) * 40, 0, -80))
			else:
				return Align_For_Aerial(agent, packet, touch.time, my_goal.direction * -100 + Vec3(0, 0, 80))
		elif bTOGT == -1 and sign(packet.game_ball.physics.velocity.y) == sign(my_goal.direction.y) and b_g_len > 6000 and abs(packet.game_ball.physics.velocity.y) > 100:
			# return Defend()
			self.carry_timer = 0
		elif my_car.boost < 70 and touch.time > 1 and pad_grab_time < (hit.hit_time + (hit.hit_position - my_goal.location).length() / max(1, hit.hit_velocity * 1.5)):
			return Grab_Boost()
		else:
			self.carry_timer = 0
		

class Wait_For_Shot(State):
	def output(self, agent, packet):
		
		hit = agent.hit
		touch = agent.touch
		
		my_car = packet.game_cars[agent.index]
		ball = packet.game_ball.physics
		
		my_goal = agent.field_info.my_goal
		
		car_loc_x_sgn = sign(my_car.physics.location.x) # car x coord sign
		def_wait_loc = Vec3(1000 * car_loc_x_sgn, my_goal.location.y + my_goal.direction.y * 200, 0) # Defense wait location for bot
		
		vec_wait_loc = (def_wait_loc - my_car.physics.location).flatten()
		dist_to_wait_loc = vec_wait_loc.length()
		def_dir = Vec3(agent.field_info.my_goal.location.x - def_wait_loc.x, 0, 0)
		
		c_boost = get_corner_boost_index(agent)
		
		if dist_to_wait_loc > 70:
			agent.controller_state = drive(agent, packet, def_wait_loc, 0.05, 0, allow_flips = True)
			agent.controller_state.boost = agent.controller_state.boost and hit.hit_time < 1 and dist_to_wait_loc > 1000
		else: # Stop if close
			car_spd_2d = my_car.physics.velocity.flatten().length()
			if car_spd_2d > 100: # Moving, assumed forward, so stop
				agent.controller_state.throttle = -1
			else: # Wait
				agent.controller_state.throttle = 0
				if Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot(def_dir.normal()) < 0.9 and car_spd_2d < 30:
					agent.maneuver = SpinJump(def_dir)
					agent.maneuver_complete = False
				
				my_car = agent.packet.game_cars[agent.index]
				for car in agent.packet.game_cars:
					if ((car.team != agent.team and (car.physics.location + car.physics.velocity * 0.5 - my_car.physics.location).length() < 350)):
						Avoid_Demo(agent, packet, car)
						break
		
		ball_intercept_time = ball_past_y_time(agent, packet, 4850)
		
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		c_b_o = (my_car.physics.location - touch.location).flatten()
		
		d_p_v = b_g_o.angle_between(c_b_o)
		d_p_v_2 = b_g_o.angle_between(agent.field_info.opponent_goal.direction)
		
		b_g_len = (ball.location + ball.velocity.flatten() - my_goal.location).length()
		
		b_g_o_len = b_g_o.length()
		
		if (touch.is_garunteed or b_g_len < 2000) and b_g_len < 6000:
			s = Defend()
			return s
		elif any((agent.team != car.team and car.has_dribble) for car in packet.game_cars):
			return Defend()
		elif ((touch.time > 4 or hit.hit_time > 3) and my_car.boost < 80 and c_boost >= 0):
			s = Grab_Boost()
			return s
		elif (d_p_v < math.pi * 0.4 and d_p_v_2 < math.pi * 0.3 and touch.time < hit.hit_time - 0.5 and b_g_o_len < 6000):
			return Take_Shot()

class Take_Shot(State):
	def output(self, agent, packet):
		touch = agent.touch
		
		my_car = packet.game_cars[agent.index]
		
		my_goal = agent.field_info.my_goal
		
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		agent.controller_state = drive(agent, packet, touch.location.flatten() + b_g_o.normal(150), touch.time, allow_flips=True)

		# tried to shoehorn jumpshot in here, failed epicly
		# if agent.touch.location.z > 120 and agent.touch.location.z <= 265:
		# 	return Jumpshot_Handler().output(agent,packet,perfect_world = False)

		if touch.time < 0.4:
			Flip_To_Ball(agent, packet)
		
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		c_b_o = (my_car.physics.location - touch.location).flatten()
		
		d_p_v = b_g_o.angle_between(c_b_o)
		d_p_v_2 = b_g_o.angle_between(agent.field_info.opponent_goal.direction)
		
		b_g_o_len = b_g_o.length()
		
		if d_p_v > math.pi * 0.5 or d_p_v_2 > math.pi * 0.4 or b_g_o_len > 6000 or not touch.is_garunteed:
			return Defend()
		elif agent.touch_type == TouchType.aerial and touch.time > 1 and touch.location.z > 300:
			return Align_For_Aerial(agent, packet, touch.time, (touch.location - agent.field_info.opponent_goal.location.flatten()).normal(130))
		

class Grab_Boost(State):
	def output(self, agent, packet):
		my_car = packet.game_cars[agent.index]
		
		c_boost = get_corner_boost_index(agent)
		
		if c_boost < 0:
			return Defend()
		
		# Collect a boost pad
		agent.controller_state = drive(agent, packet, agent.field_info.full_boosts[c_boost].location, 0.1, 0, allow_flips = True)
		
		render_star(agent, agent.field_info.full_boosts[c_boost].location, agent.renderer.yellow())
		
		if my_car.boost > 90:
			return Defend()
	

# Sets up for an aerial.
class Align_For_Aerial(State):
	def __init__(self, agent, packet, ball_time, offset):
		# Aerial offset for rough targeting purposes
		self.offset = offset
		self.time = ball_time
		self.p_time = packet.game_info.seconds_elapsed
	
	def output(self, agent, packet):
		
		delta = packet.game_info.seconds_elapsed - self.p_time
		self.p_time = packet.game_info.seconds_elapsed
		self.time -= delta
		
		car = packet.game_cars[agent.index]
		location = Get_Ball_At_T(packet, agent.ball_prediction, self.time).physics.location
		dv = delta_v(car.physics, location + self.offset, self.time, packet.game_info.world_gravity_z)
		
		impulse = impulse_velocity(packet, car.physics, location, self.time)
		
		agent.controller_state = drive(agent, packet, car.physics.location + impulse, 2)
		
		# Set up for aerial and recovery.
		if impulse.flatten().normal().dot(Vec3(1, 0, 0).align_to(car.physics.rotation)) > 0.9:
			agent.maneuver = Aerial_Takeoff(agent, packet, self)
			agent.maneuver_complete = False
			return Defend()
		
		# Abort conditions
		if self.time < 0.9 or location.z < 300:
			return Defend()
		
	

class Test_Line_Arc_Line(State):
	
	def __init__(self, agent, packet, line_arc_line, do_flip = True, execute_time = -1):
		self.line_arc_line = line_arc_line
		if execute_time < 0:
			self.execute_time = line_arc_line.calc_time()
		else:
			self.execute_time = execute_time
		self.stage = 0
		self.p_time = packet.game_info.seconds_elapsed
		self.p_car_v = 1000
		self.do_flip = do_flip
		
		# Used to make sure ball doesn't move
		self.predicted_ball_loc = Get_Ball_At_T(packet, agent.ball_prediction, self.execute_time).physics.location
	
	def reset(self, packet, line_arc_line, execute_time = -1):
		self.__init__(packet, line_arc_line, execute_time)
	
	def output(self, agent, packet):
		
		delta = packet.game_info.seconds_elapsed - self.p_time
		self.p_time = packet.game_info.seconds_elapsed
		
		self.execute_time -= delta
		
		if self.execute_time < 0:
			return Test_Line_Arc_Line_Init()
		
		if self.line_arc_line.valid:
			self.line_arc_line.render(agent)
		else:
			return Test_Line_Arc_Line_Init()
		
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
			
			agent.controller_state.throttle = (target_v - car_v + 50) * 0.03
			if abs(agent.controller_state.throttle) < 0.2:
				agent.controller_state.throttle = 0.1
			
			agent.controller_state.boost = car_v + 100 < target_v
			
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
			
			agent.controller_state.boost = car_v + 100 < target_v
			
			agent.controller_state.handbrake = abs(heading_err) > math.pi * 0.5 and car_v > 500 and abs(my_car.physics.angular_velocity.z) < 10
			
			if car_to_loc_3d.length() < car_v * delta * 2 + 50:
				self.stage += 1
			
			self.p_car_v = car_v
		
		# Check to make sure the shot is still valid
		ball = Get_Ball_At_T(packet, agent.ball_prediction, self.execute_time).physics
		
		# Path needs to be abandoned, ball has moved
		if (ball.location - self.predicted_ball_loc).length() > 50:
			return Test_Line_Arc_Line_Init()
		
		# Jump shot stuff
		ball_offset = 93
		angle = abs(math.degrees(my_car.physics.rotation.angle_to_vec(ball.location.flatten())))
		car_offset = agent.hitbox.get_offset_by_angle(angle)
		total_offset = car_offset+ball_offset
		
		targetDistance = abs((my_car.physics.location- ball.location).length())
		speed = clamp(abs(my_car.physics.velocity.length()),0.001,2300)
		
		up_v = Vec3(0, 0, 1).align_to(my_car.physics.rotation)
		
		# Project car into the future
		future_car = project_future(packet, 
			project_future(packet, Psuedo_Physics(location=my_car.physics.location,velocity=my_car.physics.velocity+up_v*300), min(self.execute_time,0.2), up_v * 1400),
		max(0, self.execute_time - 0.2))
		
		# if speed * self.execute_time < clamp(targetDistance - total_offset,0,99999) and self.execute_time < 1 and self.stage == 2:
		if future_car.location.z < ball.location.z + 30 and self.execute_time < 1 and self.stage == 2:
			agent.maneuver = Maneuver_Jump_Shot(agent, packet, self.execute_time, ball.location)
			agent.maneuver_complete = False
			return Test_Line_Arc_Line_Init()
		
	

class Path_Hit:
	def __init__(self, drive_path, time, do_flip):
		self.drive_path = drive_path
		self.time = time
		self.do_flip = do_flip

class Test_Line_Arc_Line_Init(State):
	
	def __init__(self):
		self.driver = None
	
	def calc_path(self, agent, packet):
		
		my_car = packet.game_cars[agent.index]
		current_t = packet.game_info.seconds_elapsed
		
		target_t = -1
		drive_path = None
		do_flip = True
		for i in range(agent.ball_prediction.num_slices):
			s = agent.ball_prediction.slices[i]
			ball = s.physics
			if ball.location.z < 265 and calc_hit(my_car, ball.location).time < s.game_seconds - current_t:
				target_vec = (ball.location - agent.field_info.opponent_goal.location)
				target_vec_2 = (ball.location - agent.field_info.opponent_goal.closest_point(ball.location))
				t_v = target_vec_2.copy()
				t_v.y *= 0.3
				lerp_val = clamp((2000 - t_v.length()) / 2100, 0, 1)
				attack_vec = (target_vec_2 * lerp_val + target_vec * (1 - lerp_val)).normal()
				# attack_vec = agent.field_info.my_goal.direction * -1
				dp = Line_Arc_Line(my_car, ball.location + attack_vec * 140, attack_vec * (400 + ball.location.z * 2))
				if not dp.valid or not dp.check_in_bounds():
					continue
				drive_path = dp
				target_t = s.game_seconds - current_t
				do_flip = target_vec.length() < 5000
				if drive_path.calc_time() < target_t:
					break
		
		return Path_Hit(drive_path, target_t, do_flip)
		
	
	def output(self, agent, packet):
		
		my_car = packet.game_cars[agent.index]
		drive_path = self.calc_path(agent, packet)
		if drive_path.drive_path is None:
			Test_Drive_Goal().output(agent, packet)
		else:
			self.driver = Test_Line_Arc_Line(agent, packet, drive_path.drive_path, execute_time = drive_path.time, do_flip = drive_path.do_flip)
			self.driver.output(agent, packet)
			
			car_to_p1 = (drive_path.drive_path.p1 - my_car.physics.location).inflate()
			
			if Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot(car_to_p1.normal()) > 0.95 and car_to_p1.length() < 700:
				return self.driver
		
	

class Test_Drive_Goal(State):
	def output(self, agent, packet):
		my_car = packet.game_cars[agent.index]
		agent.controller_state = drive(agent, packet, agent.field_info.my_goal.location, 2)
		if (my_car.physics.location - agent.field_info.my_goal.location).length() < 1000 or (my_car.physics.location - packet.game_ball.physics.location).length() > 3000:
			return Test_Line_Arc_Line_Init()

