
from Structs import Vec3
from Utils import *
from Actions import *

# A state either returns None (for allowing the program to select state) or a state which will be run on the next get_output
# Can also return itself to lock its own state

class State:
	def output(self, agent, packet):
		pass

class Recovery(State):
	def __init__(self, prev_state):
		self.prev_state = prev_state
	def output(self, agent, packet):
		Align_Car_To(agent, packet, packet.game_cars[agent.index].physics.velocity.flatten(), Vec3(0, 0, 1))
		return self.prev_state if packet.game_cars[agent.index].has_wheel_contact else self

class Carry_Ball(State):
	def output(self, agent, packet):
		Dribble(agent, packet, agent.field_info.opponent_goal.location)
		my_car = packet.game_cars[agent.index]
		if agent.packet.game_ball.physics.location.z < 120 or (agent.packet.game_ball.physics.location - my_car.physics.location).length() > 200 or not my_car.has_wheel_contact:
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
		
		if (bTOGT != -1 or (ball.location + ball.velocity.flatten() - my_goal.location).length() < 1000 or agent.touch_type != TouchType.ground) and touch.time < 0.4:
			Flip_To_Ball(agent, packet)
		
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		c_b_o = (my_car.physics.location - touch.location).flatten()
		
		d_p_v = b_g_o.angle_between(c_b_o)
		d_p_v_2 = b_g_o.angle_between(agent.field_info.opponent_goal.direction)
		
		b_g_o_len = b_g_o.length()
		
		b_g_len = (touch.location - my_goal.location).flatten().length()
		
		c_boost = get_corner_boost_index(agent)
		
		# We keep our own dribble timer in order to be more restrictive than the dribble tracker.
		if abs(160 - ball.location.z) < 30 and (ball.location - my_car.physics.location).length() < 200:
			self.carry_timer += agent.delta
			if self.carry_timer > 1:
				return Carry_Ball()
		elif any((agent.team != car.team and car.has_dribble) for car in packet.game_cars):
			self.carry_timer = 0
		elif (d_p_v < math.pi * 0.4 and d_p_v_2 < math.pi * 0.3 and touch.time < hit.hit_time - 0.5 and b_g_o_len < 6000):
			return Take_Shot()
		elif bTOGT == -1 and sign(packet.game_ball.physics.velocity.y) == sign(my_goal.direction.y) and b_g_len > 6000 and abs(packet.game_ball.physics.velocity.y) > 100:
			return Defend()
		elif ((touch.time > 2 and hit.hit_time > 3) and my_car.boost < 70 and c_boost >= 0):
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
		if touch.time < 0.4:
			Flip_To_Ball(agent, packet)
		
		b_g_o = (touch.location - agent.field_info.opponent_goal.location).flatten()
		c_b_o = (my_car.physics.location - touch.location).flatten()
		
		d_p_v = b_g_o.angle_between(c_b_o)
		d_p_v_2 = b_g_o.angle_between(agent.field_info.opponent_goal.direction)
		
		b_g_o_len = b_g_o.length()
		
		if d_p_v > math.pi * 0.5 or d_p_v_2 > math.pi * 0.4 or b_g_o_len > 6000 or not touch.is_garunteed:
			return Defend()
		

class Grab_Boost(State):
	def output(self, agent, packet):
		my_car = packet.game_cars[agent.index]
		
		c_boost = get_corner_boost_index(agent)
		
		# Collect a boost pad
		agent.controller_state = drive(agent, packet, agent.field_info.boosts[c_boost].location, 0.1, 0, allow_flips = True)
		
		if my_car.boost > 90 or agent.hit.hit_time < 0.5 or c_boost < 0:
			return Defend()
	
