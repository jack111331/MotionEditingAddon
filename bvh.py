import bvh_lex
import ply.lex as lex
class BVHNode:
	def __init__(self, ):
		self.joint_name = ""
		self.channels = 0
		self.channel_layout_assign = []
		self.in_motion_start_idx = 0
		self.parent = None

class Frame:
	def __init__(self):
		self.motion_parameter_list = []

class Motion:
	def __init__(self):
		self.frame_list = []

class BVHParser:
	def __init__(self, bvh_filename):
		self.bone_list = []
		self.joint_parameter_amount = 0
		self.motion_list = []
		bvh_lexer = bvh_lex.BVHLexer()
		try:
			with open(bvh_filename) as f:
				lex.runmain(lexer=bvh_lexer, data=f.read())
		except IndexError:
			print('Can\'t open bvh file')

	def add_motion(self, bvh_filename):
		return

if __name__ == '__main__':
	parser = BVHParser("bvh_example/BVH.bvh")
