
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
		car = packet.game_cars[agent.index]
		
		if car.physics.location.z > -car.physics.velocity.z:
			vec = car.physics.velocity.flatten().normal() + Vec3(0, 0, -1)
			Align_Car_To(agent, packet, vec, Vec3(0, 0, 1))
			agent.controller_state.boost = Vec3(1, 0, 0).align_to(car.physics.rotation).dot(vec.normal()) > 0.9
		else:
			Align_Car_To(agent, packet, car.physics.velocity.flatten(), Vec3(0, 0, 1))
		
		
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
		
		bTOGT = ball_in_my_goal_time(agent, packet)
		
		touch = agent.hit_package.ground_touch
		
		current_time = packet.game_info.seconds_elapsed
		
		my_car = packet.game_cars[agent.index]
		cbl = (agent.packet.game_ball.physics.location - my_car.physics.location).length()
		
		if cbl < 200:
			Dribble(agent, packet, agent.field_info.opponent_goal.location)
			
			if agent.hit.hit_time < 0.5:
				Enter_Flick(agent, packet, (agent.field_info.opponent_goal.location - my_car.physics.location).normal())
				return Defend()
			
		else:
			agent.controller_state = drive(agent, packet, touch.location + Vec3(-10).align_to(my_car.physics.rotation), touch.time)
			
			if agent.hit_package.ground_touch.time > agent.hit.hit_time - 0.5:
				return Defend()
			
		
		if agent.packet.game_ball.physics.location.z < 120 or not my_car.has_wheel_contact or bTOGT != -1:
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
		
		dest = touch.location.flatten()
		
		my_car = packet.game_cars[agent.index]
		ball = packet.game_ball.physics
		
		my_goal = agent.field_info.my_goal
		
		# if agent.field_info.my_goal.direction.dot((packet.game_ball.physics.location - my_car.physics.location).normal()) < -0.1 and (abs(dest.x) < 1200 or dest.y * my_goal.direction.y > -4000):
			# dest += Vec3(200 * sign(my_car.physics.location.x - packet.game_ball.physics.location.x), -agent.field_info.my_goal.direction.y * 500, 0)
			# agent.controller_state = drive(agent, packet, dest, touch.time - 0.2, allow_flips=True)
		# else:
			# agent.controller_state = drive(agent, packet, dest, touch.time, allow_flips=True)
		
		JumpShot_Handler(agent, packet)
		
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
		
		pad_grab_time_2 = calc_hit(my_car, agent.field_info.full_boosts[c_boost].location).time + Time_to_Pos((agent.field_info.full_boosts[c_boost].location - touch.location).length(), 0, 100).time if c_boost >= 0 else 10000
		
		if agent.hit_package.ground_touch.time < hit.hit_time - 1 and agent.hit_package.ground_touch.time < 1.2 and ball.location.z > 120 and bTOGT == -1:
			return Carry_Ball()
		elif (d_p_v < math.pi * 0.4 and d_p_v_2 < math.pi * 0.3 and touch.time < hit.hit_time - 0.5 and b_g_o_len < 6000):
			return Take_Shot()
		elif agent.touch_type == TouchType.aerial and touch.is_garunteed and touch.time > 1 and touch.location.z > 300:
			if my_goal.direction.dot(my_car.physics.location - touch.location) > 0.0:
				return Align_For_Aerial(agent, packet, my_goal.direction * -90 + Vec3(sign(my_car.physics.location.x - touch.location.x) * 40, 0, -80))
			else:
				return Align_For_Aerial(agent, packet, my_goal.direction * -100 + Vec3(0, 0, -80))
		elif my_car.boost < 70 and touch.time > 1 and ((pad_grab_time < (hit.hit_time + (hit.hit_position - my_goal.location).length() / max(1, hit.hit_velocity * 1.75)) - 0.5 and touch.time > hit.hit_time) or pad_grab_time_2 < touch.time) and (pad_grab_time < bTOGT or bTOGT == -1) and touch.time < hit.hit_time + 2:
			return Grab_Boost()
		

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
	
	def __init__(self):
		self.driver = None
	
	def output(self, agent, packet):
		
		drive_path = calc_path(Shot_On_Goal(), agent, packet)
		
		my_car = packet.game_cars[agent.index]
		touch = agent.touch
		
		if drive_path.drive_path is None or drive_path.time > agent.hit_package.flip_touch.time * 1.5 or drive_path.time > agent.hit.hit_time:
			
			JumpShot_Handler(agent, packet)
			
		else:
			self.driver = Line_Arc_Line_Driver(agent, packet, drive_path.drive_path, execute_time = drive_path.time)
			self.driver.update(agent, packet)
			
			car_to_p1 = (drive_path.drive_path.p1 - my_car.physics.location).inflate()
			
			if Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot(car_to_p1.normal()) > 0.95 and car_to_p1.length() < 1000:
				agent.maneuver_complete = False
				agent.maneuver = self.driver
			
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		c_b_o = (my_car.physics.location - touch.location).flatten()
		
		d_p_v = b_g_o.angle_between(c_b_o)
		d_p_v_2 = b_g_o.angle_between(agent.field_info.opponent_goal.direction)
		
		b_g_o_len = b_g_o.length()
		
		if d_p_v > math.pi * 0.5 or d_p_v_2 > math.pi * 0.4 or b_g_o_len > 6000 or not touch.is_garunteed:
			return Defend()
		elif agent.touch_type == TouchType.aerial and touch.time > 1 and touch.location.z > 300:
			return Align_For_Aerial(agent, packet, (touch.location - agent.field_info.opponent_goal.location.flatten()).normal(130))
		

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
	def __init__(self, agent, packet, offset):
		# Aerial offset for rough targeting purposes
		self.offset = offset
		self.p_time = packet.game_info.seconds_elapsed
	
	def output(self, agent, packet):
		
		touch = agent.hit_package.air_touch
		self.time = touch.time
		
		delta = packet.game_info.seconds_elapsed - self.p_time
		self.p_time = packet.game_info.seconds_elapsed
		
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
		if self.time < 0.7 or location.z < 265:
			return Defend()
		
	

class Test_Line_Arc_Line_Init(State):
	
	def __init__(self):
		self.driver = None
	
	def output(self, agent, packet):
		
		my_car = packet.game_cars[agent.index]
		drive_path = calc_path(Shot_On_Goal(), agent, packet)
		
		if drive_path.drive_path is None:
			drive_path = calc_path(Shot_In_Direction(agent.field_info.my_goal.direction), agent, packet)
		
		if drive_path.drive_path is None:
			drive_path = calc_path(Shot_To_Side(), agent, packet)
		
		if drive_path.drive_path is None:
			Test_Drive_Goal().output(agent, packet)
		else:
			self.driver = Line_Arc_Line_Driver(agent, packet, drive_path.drive_path, execute_time = drive_path.time)
			self.driver.update(agent, packet)
			
			car_to_p1 = (drive_path.drive_path.p1 - my_car.physics.location).inflate()
			
			if Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot(car_to_p1.normal()) > 0.95 and car_to_p1.length() < 700:
				agent.maneuver_complete = False
				agent.maneuver = self.driver
		
	

class Test_Drive_Goal(State):
	def output(self, agent, packet):
		my_car = packet.game_cars[agent.index]
		agent.controller_state = drive(agent, packet, agent.field_info.my_goal.location, 2)
		if (my_car.physics.location - agent.field_info.my_goal.location).length() < 1000 or (my_car.physics.location - packet.game_ball.physics.location).length() > 3000:
			return Test_Line_Arc_Line_Init()

