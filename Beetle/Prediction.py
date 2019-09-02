import time

from Structs import Packet, BallPrediction, FieldInfo, Hit_Package
from Utils import Hit_Prediction

from rlbot.utils.structures.game_interface import GameInterface
from rlbot.utils.structures.game_data_struct import GameTickPacket, FieldInfoPacket
from rlbot.utils.structures.ball_prediction_struct import BallPrediction as RLBotBallPrediction
from rlbot.botmanager.bot_helper_process import BotHelperProcess

from rlbot.utils.logging_utils import get_logger

from queue import Queue

# This is the part that just isn't working at all. The queues need to be the same. I have created a unique key, but I can't get the bot to access the queue that the BotHelperProcess creates, and I can't seem to create my own queue and use that. Everything else should either function or be really close to functioning.
# Also, may want to remove the unpackaging of the BallPrediction object from Beetle since it won't be used as much.
hit_prediction_queues = {}
def get_queue(key):
	if not key in hit_prediction_queues:
		print("Create hit prediction queue: "+key)
		hit_prediction_queues[key] = Queue()
	return hit_prediction_queues.get(key)

class Hit_Predictor(BotHelperProcess):
	
	def __init__(self, agent_metadata_queue, quit_event, options):
		super().__init__(agent_metadata_queue, quit_event, options)
		self.logger = get_logger('Beetle')
		self.game_interface = GameInterface(self.logger)
		self.bot_agent = None
	
	def start(self):
		self.game_interface.load_interface()
		
		field_info = FieldInfoPacket()
		self.game_interface.update_field_info_packet(field_info)
		
		self.packet = None
		self.gtp = GameTickPacket()
		self.bp = RLBotBallPrediction()
		
		# Wait for agent to boot up and get its metadata
		single_agent_metadata = self.metadata_queue.get()
		self.index = single_agent_metadata.index
		self.team = single_agent_metadata.team
		
		# This is where the queues would be linked. Currently this method utterly fails and needs to be fixed.
		self.hit_prediction_queue = get_queue(single_agent_metadata.helper_process_request.key)
		self.logger.debug(single_agent_metadata.helper_process_request.key)
		
		self.field_info = FieldInfo(self, field_info)
		
		self.p_time = -1
		
		# Allow for quitting cleanly
		while not self.quit_event.is_set():
			
			# Update field info
			self.game_interface.update_live_data_packet(self.gtp)
			self.game_interface.update_ball_prediction(self.bp)
			
			if self.gtp.game_info.seconds_elapsed == self.p_time:
				continue
			
			self.p_time = self.gtp.game_info.seconds_elapsed
			
			self.packet = Packet(self.gtp)
			self.ball_prediction = BallPrediction(self.bp)
			
			my_car = self.packet.game_cars[self.index]
			my_goal = self.field_info.my_goal
			
			hit = Hit_Prediction(self, self.packet)
			touch1 = hit.get_earliest_touch(self, self.packet, my_car, 120, my_goal.direction * -30)
			touch2 = hit.get_earliest_touch(self, self.packet, my_car, 180, my_goal.direction * -40)
			
			self.hit_prediction_queue.put_nowait(Hit_Package(hit, touch1, touch2))
			
		
		self.logger.debug("Quit Beetle Prediction services")
	

