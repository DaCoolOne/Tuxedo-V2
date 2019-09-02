import math
import os
import time
import multiprocessing

from rlbot.agents.base_agent import BaseAgent
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.botmanager.helper_process_request import HelperProcessRequest

from Structs import *
from Utils import *
from Actions import *
from States import *

from Prediction import get_queue

class Beetle(BaseAgent):
	
	def __init__(self, name, team, index):
		super().__init__(name, team, index)
		self.key = f"69 420 {index}"
		self.hit_prediction_queue = get_queue(self.key)
	
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
		self.state = Defend()
		self.touch_type = TouchType.ground
		self.dribble_tracker = Dribble_Tracker(self)
		self.hit = None
		self.touch = None
		self.field_info = FieldInfo(self, self.get_field_info())
		# self.communication_queue = 

	def Preprocessing(self, gtp: GameTickPacket):
		self.packet = Packet(gtp)
		self.ball_prediction = BallPrediction(self.get_ball_prediction_struct())
		
		self.delta = self.packet.game_info.seconds_elapsed - self.p_time
		self.p_time = self.packet.game_info.seconds_elapsed
		
		self.dribble_tracker.set_packet(self.packet)
		
		if not self.was_active and self.packet.game_info.is_kickoff_pause:
			self.maneuver = Kickoff(self, self.packet)
			self.maneuver_complete = False
			self.state = Defend()
		
		self.was_active = self.packet.game_info.is_round_active
		
	
	def get_helper_process_request(self):
		fp = os.path.dirname(os.path.realpath(__file__))
		self.helper_process = HelperProcessRequest(python_file_path=fp+"\\Prediction.py",key=self.key)
		return self.helper_process
	
	def get_output(self, gtp: GameTickPacket):
		
		self.Preprocessing(gtp)
		
		if self.delta == 0:
			return MyControllerState().get()
		
		my_car = self.packet.game_cars[self.index]
		
		self.renderer.begin_rendering()
		
		self.controller_state = MyControllerState()
		
		my_goal = self.field_info.my_goal
		
		if not self.maneuver_complete:
			self.maneuver_complete = self.maneuver.update(self, self.packet)
			self.renderer.draw_string_3d(my_car.physics.location.UI_Vec3(), 2, 2, type(self.maneuver).__name__, self.renderer.red())
		else:
			
			# Can't get it with flips, check for aerials.
			# if touch.time > hit.hit_time + 0.25:
				# touch = hit.get_earliest_touch(self, self.packet, self.packet.game_cars[self.index], offset = self.field_info.my_goal.direction * -60 + Vec3(0, 80, 0))
				# if touch.location.z > 160:
					# self.touch_type = TouchType.aerial
				# else:
					# self.touch_type = TouchType.flip
			
			# Calculate a touch with preference for stuff we don't have to jump for.
			# hit = Hit_Prediction(self, self.packet)
			# touch = hit.get_earliest_touch(self, self.packet, my_car, 120, my_goal.direction * -30)
			
			# self.touch_type = TouchType.ground
			
			# Can't get it with normal touch, check for flips.
			# if touch.time > hit.hit_time - 0.25:
				# self.touch_type = TouchType.flip
				# touch = hit.get_earliest_touch(self, self.packet, my_car, 180, my_goal.direction * -40)
			
			# Calculate a touch with preference for stuff we don't have to jump for.
			
			# Wait for hit to be created
			while self.hit is None and self.hit_prediction_queue.empty():
				time.sleep(0.05)
			
			# Update latest hits (Use while loop so that we always have latest data)
			while not self.hit_prediction_queue.empty():
				hit_package = self.helper_process.metadata_queue.get()
				
				hit = hit_package.hit
				touch = hit_package.ground_touch
				self.touch_type = TouchType.ground
				
				# Can't get it with normal touch, check for flips.
				if touch.time > hit.hit_time - 0.25:
					self.touch_type = TouchType.flip
					touch = hit_package.flip_touch
				
				self.hit = hit
				self.touch = touch
			
			next_state = self.state.output(self, self.packet)
			if next_state is None:
				if not my_car.has_wheel_contact:
					next_state = Recovery(self.state)
				else:
					next_state = self.state
			self.state = next_state
			
			self.renderer.draw_string_3d((my_car.physics.location).UI_Vec3(), 2, 2, type(self.state).__name__, self.renderer.yellow())
			
		
		self.renderer.end_rendering()
		
		self.controller_state.boost = self.controller_state.boost and my_car.physics.velocity.length() < 2275
		self.controller_state.jump = self.controller_state.jump and self.packet.game_info.is_round_active
		
		return self.controller_state.get()
	
