import math

from rlbot.agents.base_agent import BaseAgent
from rlbot.utils.structures.game_data_struct import GameTickPacket

from Structs import *
from Utils import *
from Actions import *

class Beetle(BaseAgent):
	
	def initialize_agent(self):
		self.packet = None
		self.field_info = None
		self.controller_state = None
		self.flip_timer = 0
		self.flipping = False
		self.delta = 0
		self.p_time = 0
		self.score_total = 0
		self.goal_since_last_frame = False
		self.going_for_intercept = False
		self.ball_prediction = None
		self.offensive_mode = 0 # Makes the bot offensive :P
		self.maneuver_complete = True
		self.maneuver: Maneuver = None
		self.getting_boost = False
		self.was_active = False
		self.taking_shot = False
	
	def get_output(self, gtp: GameTickPacket):
		
		self.packet = Packet(gtp)
		
		self.ball_prediction = BallPrediction(self)
		
		self.delta = self.packet.game_info.seconds_elapsed - self.p_time
		self.p_time = self.packet.game_info.seconds_elapsed
		
		if not self.was_active and self.packet.game_info.is_kickoff_pause:
			self.maneuver = Kickoff(self, self.packet)
			self.maneuver_complete = False
		
		self.was_active = self.packet.game_info.is_round_active
		
		if self.field_info == None:
			self.field_info = FieldInfo(self, self.get_field_info())
		
		if self.delta == 0:
			return MyControllerState().get()
		
		my_car = self.packet.game_cars[self.index]
		
		self.renderer.begin_rendering()
		
		self.controller_state = MyControllerState()
		
		if not self.maneuver_complete:
			self.maneuver_complete = self.maneuver.update(self, self.packet)
			self.renderer.draw_string_3d(my_car.physics.location.UI_Vec3(), 2, 2, "Maneuver Lock", self.renderer.yellow())
		elif not my_car.has_wheel_contact:
			# Recoveries and half flips.
			Align_Car_To(self, self.packet, my_car.physics.velocity.flatten(), Vec3(0, 0, 1))
		elif self.offensive_mode > 1.0:
			self.renderer.draw_string_3d(my_car.physics.location.UI_Vec3(), 2, 2, "Dribble", self.renderer.blue())
			if self.packet.game_ball.physics.location.z < 120 or (self.packet.game_ball.physics.location - my_car.physics.location).length() > 200 or not my_car.has_wheel_contact:
				self.offensive_mode = 0
			else:
				Dribble(self, self.packet, self.field_info.opponent_goal.location)
		else:
			car_spd_2d = my_car.physics.velocity.flatten().length()
			ball = self.packet.game_ball.physics #ex. ball.location.y
			
			car_loc_x_sgn = sign(my_car.physics.location.x) # car x coord sign
			def_wait_loc = Vec3(1000 * car_loc_x_sgn, self.field_info.my_goal.location.y + self.field_info.my_goal.direction.y * 200, 0) # Defense wait location for bot
			wait_for_shot = True
			
			# Okay, time to add some of my utils >:)
			
			need_flip = False
			need_aerial = False
			
			# Calculate a touch with preference for stuff we don't have to jump for.
			hit = Hit_Prediction(self, self.packet)
			touch = hit.get_earliest_touch(self, self.packet, self.packet.game_cars[self.index], 120, self.field_info.my_goal.direction * -30)
			# Can't get it with normal touch, check for flips.
			if touch.time > hit.hit_time - 0.25:
				need_flip = True
				touch = hit.get_earliest_touch(self, self.packet, self.packet.game_cars[self.index], 180, self.field_info.my_goal.direction * -40)
			# Can't get it with flips, check for aerials.
			# if not touch.can_save:
				# touch = hit.get_earliest_touch(self, self.packet, self.packet.game_cars[self.index], offset = self.field_info.my_goal.direction * -40)
				# need_aerial = True
			
			hit.draw_path(self.renderer, self.renderer.yellow())
			
			b_g_len = (ball.location + ball.velocity.flatten() - self.field_info.my_goal.location).length()
			b_g_o = (touch.location - self.field_info.opponent_goal.location).flatten()
			b_g_o_len = Vec3(b_g_o.x, b_g_o.y * 0.25).length()
			
			bTOGT = ball_in_my_goal_time(self, self.packet)
			if (b_g_o_len < 1200 and touch.time < hit.hit_time + 0.5) or self.taking_shot:
				# Take the shot!
				self.taking_shot = b_g_o_len < 1500 and touch.is_garunteed
				
				self.controller_state = drive(self, self.packet, touch.location.flatten() + b_g_o.normal(150), touch.time, allow_flips=True)
				if touch.time < 0.4:
					Flip_To_Ball(self, self.packet)
				
				self.renderer.draw_string_3d(my_car.physics.location.UI_Vec3(), 2, 2, "Taking Shot", self.renderer.green())
				
				wait_for_shot = False
				
			# If ball is heading into our goal
			elif bTOGT != -1 or sign(self.packet.game_ball.physics.velocity.y) != sign(self.field_info.my_goal.direction.y) or b_g_len < 6000 or abs(self.packet.game_ball.physics.velocity.y) < 100:
				# self.renderer.draw_string_2d(20, 60, 3, 3, str(bTOGT), self.renderer.team_color()) # Predicted time of goal
				# self.renderer.draw_string_2d(20, 120, 3, 3, str(bTOGT - self.p_time), self.renderer.team_color()) # Predicted time until goal
				render_star(self, ball_loc_at_time(self, self.packet, bTOGT), self.renderer.red())
				# cross_y_time = ball_past_y_time(self, self.packet, 5030) # Time when ball starts entering my net
				# ball_loc_net_start = ball_loc_at_time(self, self.packet, cross_y_time) # Where the ball is when it begins to enter my net
				#render_star(self, ball_loc_net_start, self.renderer.orange())
				
				ball_intercept_time = ball_past_y_time(self, self.packet, 4850) #4970
				# ball_intercept_loc = ball_loc_at_time(self, self.packet, ball_intercept_time) # Where the ball is when it begins to enter my net
				# car_intercept_loc = ball_intercept_loc.flatten() # For now
				# travel_dist = (car_intercept_loc - my_car.physics.location.flatten()).length()
				# drive_time = line_drive_duration(car_spd_2d, travel_dist, my_car.boost)
				# spare_time = ball_intercept_time - self.p_time - drive_time
				
				render_star(self, touch.location, self.renderer.yellow())
				# self.renderer.draw_string_2d(20, 60, 3, 3, str(ball_intercept_time - self.packet.game_info.seconds_elapsed), self.renderer.team_color())
				
				if (ball_intercept_time - touch.time - self.packet.game_info.seconds_elapsed < 0.1 and b_g_len < 5000) or (touch.is_garunteed and not self.getting_boost) or b_g_len < 2000:
					self.going_for_intercept = True
				
				if self.going_for_intercept:
					wait_for_shot = False
					dest = touch.location.flatten() # - self.field_info.my_goal.direction * 40 # Location of the earliset touch we can achieve
					
					if self.field_info.my_goal.direction.dot((self.packet.game_ball.physics.location - my_car.physics.location).normal()) < -0.1:
						dest += Vec3(200 * sign(my_car.physics.location.x - self.packet.game_ball.physics.location.x), -self.field_info.my_goal.direction.y * 500, 0)
						self.controller_state = drive(self, self.packet, dest, touch.time - 0.2, allow_flips=True)
					# elif self.packet.game_ball.physics.location.z > 120:
						# self.controller_state = drive_catch(self, self.packet, Get_Ball_At_T(self.packet, self.ball_prediction, touch.time).physics.location, touch.time)
					else:
						self.controller_state = drive(self, self.packet, dest, touch.time, allow_flips=True)
					
					if (bTOGT != -1 or b_g_len < 1000 or need_flip) and touch.time < 0.4:
						Flip_To_Ball(self, self.packet)
					
			else:
				self.going_for_intercept = False
			
			if wait_for_shot: # This is really bad, but works for testing
				vec_wait_loc = (def_wait_loc - my_car.physics.location).flatten()
				dist_to_wait_loc = vec_wait_loc.length()
				def_dir = Vec3(self.field_info.my_goal.location.x - def_wait_loc.x, 0, 0)
				
				c_boost = self.get_corner_boost_index()
				
				if ((touch.time > 4 or hit.hit_time > 3) and my_car.boost < 80 and c_boost >= 0) or self.getting_boost:
					self.getting_boost = my_car.boost < 80
					
					# Collect a boost pad
					self.controller_state = drive(self, self.packet, self.field_info.boosts[c_boost].location, 0.1, 0, allow_flips = True)
				elif dist_to_wait_loc > 70:
					self.controller_state = drive(self, self.packet, def_wait_loc, 0.05, 0, allow_flips = True)
					self.controller_state.boost = self.controller_state.boost and hit.hit_time < 1 and dist_to_wait_loc > 1000
				else: # Stop if close
					if car_spd_2d > 100: # Moving, assumed forward, so stop
						self.controller_state.throttle = -1
					else: # Wait
						self.controller_state.throttle = 0
						if Vec3(1, 0, 0).align_to(my_car.physics.rotation).dot(def_dir.normal()) < 0.9 and car_spd_2d < 30:
							self.maneuver = SpinJump(def_dir)
							self.maneuver_complete = False
						
						my_car = self.packet.game_cars[self.index]
						for car in self.packet.game_cars:
							if ((car.team != self.team and (car.physics.location + car.physics.velocity * 0.5 - my_car.physics.location).length() < 350)):
								Avoid_Demo(self, self.packet, car)
								break
			
			#render_star(self, ball_loc_at_time(self, self.packet, self.p_time + 2.0), self.renderer.blue())
			
			if abs(160 - self.packet.game_ball.physics.location.z) < 30 and (self.packet.game_ball.physics.location - my_car.physics.location).length() < 200:
				self.offensive_mode += self.delta
			else:
				self.offensive_mode = 0
			
			"""
			val1 = self.get_ball_prediction_struct().slices[10].game_seconds - self.get_ball_prediction_struct().slices[9].game_seconds
			print("###")
			print(self.p_time)
			print(str(val1))
			print(self.get_ball_prediction_struct().slices[1].game_seconds)
			print(1.0 / 60.0)
			"""
			
		
		# self.renderer.draw_string_3d(self.packet.game_ball.physics.location.UI_Vec3(), 2, 2, str(self.packet.game_ball.physics.location.z), self.renderer.white())
		
		self.renderer.end_rendering()
		
		self.controller_state.boost = self.controller_state.boost and not my_car.is_super_sonic
		self.controller_state.jump = self.controller_state.jump and self.packet.game_info.is_round_active
		
		return self.controller_state.get()
	
	# Gets the boost that we will collect
	def get_corner_boost_index(self):
		
		my_car = self.packet.game_cars[self.index]
		
		s = sign(clamp_abs(my_car.physics.location.y, 4000) - self.field_info.my_goal.location.y)
		s2 = sign(my_car.physics.location.x)
		
		max_l = 0
		
		max_i = -1
		
		for i, boost in enumerate(self.field_info.boosts):
			if sign(clamp_abs(my_car.physics.location.y, 4000) - boost.location.y) == s and sign(boost.location.x) == s2 and self.packet.game_boosts[i].is_active and boost.is_full_boost:
				if (boost.location.y - self.field_info.my_goal.location.y) * s > max_l:
					max_i = i
					max_l = (boost.location.y - self.field_info.my_goal.location.y) * s
			
		
		return max_i
		
	
