import bvh_lex
class BVHNode:
	channel_layout = ['XPOSITION', 'YPOSITION',
					  'ZPOSITION', 'XROTATION', 'YROTATION', 'ZROTATION']

	def __init__(self):
		self.joint_name = ""
		self.channels = 0
		self.channel_layout_assign = []
		self.in_motion_start_idx = 0
		self.offset = [0.0, 0.0, 0.0]
		self.parent = None

	def assign_parent(self, parent):
		self.parent = parent

	def parse_joint_name(self, bvh_lex):
		self.joint_name = bvh_lex.token().value

	def parse_offset(self, bvh_lex):
		self.offset[0] = bvh_lex.token().value
		self.offset[1] = bvh_lex.token().value
		self.offset[2] = bvh_lex.token().value
	def parse_channel_layout(self, bvh_lex, joint_parameter_amount):
		self.channels = bvh_lex.token().value
		for i in range(self.channels):
			type = bvh_lex.token().type
			for j in range(len(BVHNode.channel_layout)):
				if type == BVHNode.channel_layout[j]:
					self.channel_layout_assign.append(j)
					break
		self.in_motion_start_idx = joint_parameter_amount
		return self.channels

class AggregateFrame:
	def __init__(self):
		self.node_list = None
		self.frame = None
	def assign_node_list(self, node_list):
		self.node_list = node_list
	def assign_frame(self, frame):
		self.frame = frame

class Frame:
	def __init__(self):
		self.motion_parameter_list = []

	def parse_frame(self, bvh_lex, joint_parameter_amount):
		for i in range(joint_parameter_amount):
			self.motion_parameter_list.append(bvh_lex.token().value)

class Motion:
	def __init__(self):
		self.frame_list = []
		self.frame_amount = 0
		self.frame_time = 0.0

	def parse_motion(self, bvh_lex, joint_parameter_amount):
		while True:
			token = bvh_lex.token()
			if token.type == "FRAMES":
				self.frame_amount = bvh_lex.token().value
			elif token.type == "FRAMETIME":
				self.frame_time = bvh_lex.token().value
				break

		for i in range(self.frame_amount):
			frame = Frame()
			frame.parse_frame(bvh_lex, joint_parameter_amount)
			self.frame_list.append(frame)

class BVHParser:
	def __init__(self, bvh_filename):
		self.node_list = []
		self.joint_parameter_amount = 0
		self.motion_list = []
		bvh_lexer = bvh_lex.BVHLexer()
		try:
			with open(bvh_filename) as f:
				bvh_lexer.input(f.read())
		except IndexError:
			print('Can\'t open bvh file')

		layer_stack = []
		while True:
			token = bvh_lexer.token()
			if token == None:
				break

			if token.type == "HIERARCHY":
				print('This is a BVH file')
			elif token.type == "ROOT":
				node = BVHNode()
				node.parse_joint_name(bvh_lexer)
				self.node_list.append(node)
				layer_stack.append(node)
			elif token.type == "OFFSET":
				self.node_list[-1].parse_offset(bvh_lexer)
			elif token.type == "CHANNELS":
				joint_parameter_amount = self.node_list[-1].parse_channel_layout(bvh_lexer, self.joint_parameter_amount)
				self.joint_parameter_amount += joint_parameter_amount
			elif token.type == "JOINT":
				node = BVHNode()
				node.assign_parent(layer_stack[-1])
				node.parse_joint_name(bvh_lexer)
				layer_stack.append(node)
				self.node_list.append(node)
			elif token.type == "ENDJOINT":
				node = BVHNode()
				node.assign_parent(layer_stack[-1])
				node.joint_name = "Dummy"
				layer_stack.append(node)
				self.node_list.append(node)
			elif token.type == "LPAREN":
				pass
			elif token.type == "RPAREN":
				layer_stack.pop()
			elif token.type == "MOTION":
				motion = Motion()
				self.motion_list.append(motion)
				motion.parse_motion(bvh_lexer, self.joint_parameter_amount)
			else:
				print("Unidentified token")

	def add_motion(self, bvh_filename):
		bvh_lexer = bvh_lex.BVHLexer()
		try:
			with open(bvh_filename) as f:
				bvh_lexer.input(f.read())
		except IndexError:
			print('Can\'t open bvh file')

		while True:
			token = bvh_lexer.token()
			if token == None:
				break

			if token.type == "MOTION":
				motion = Motion()
				self.motion_list.append(motion)
				motion.parse_motion(bvh_lexer, self.node_list)
			else:
				print("Unidentified token")

if __name__ == '__main__':
	parser = BVHParser("bvh_example/BVH.bvh")
