from . import bvh_lex
import bpy
from mathutils import Vector


class BVHNode:
    channel_layout = ['XPOSITION', 'YPOSITION',
                      'ZPOSITION', 'XROTATION', 'YROTATION', 'ZROTATION']

    def __init__(self):
        self.joint_name = ""
        self.channels = 0
        self.channel_layout_assign = []
        self.in_motion_start_idx = 0
        self.rest_head_local = Vector((0.0, 0.0, 0.0))
        self.rest_head_world = Vector((0.0, 0.0, 0.0))
        self.rest_tail_local = None
        self.rest_tail_world = None
        self.parent = None
        self.children = []
        self.has_loc = False
        self.has_rot = False
        self.temp = None

    def assign_parent(self, parent):
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

    def parse_joint_name(self, bvh_lex):
        self.joint_name = bvh_lex.token().value

    def parse_offset(self, bvh_lex):
        self.rest_head_local = Vector((bvh_lex.token().value, bvh_lex.token().value, bvh_lex.token().value))
        if self.parent == None:
            self.rest_head_world = self.rest_head_local
        else:
            self.rest_head_world = self.parent.rest_head_world + self.rest_head_local

    def parse_channel_layout(self, bvh_lex, joint_parameter_amount):
        self.channels = bvh_lex.token().value
        for i in range(self.channels):
            type = bvh_lex.token().type
            for j in range(len(BVHNode.channel_layout)):
                if type == BVHNode.channel_layout[j]:
                    self.channel_layout_assign.append(j)
                    break
        self.in_motion_start_idx = joint_parameter_amount
        self.has_loc = {0, 1, 2} in self.channel_layout_assign
        self.has_rot = {3, 4, 5} in self.channel_layout_assign
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
        end_joint_encounter = False
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
                layer_stack[-1].parse_offset(bvh_lexer)
            elif token.type == "CHANNELS":
                joint_parameter_amount = layer_stack[-1].parse_channel_layout(bvh_lexer, self.joint_parameter_amount)
                self.joint_parameter_amount += joint_parameter_amount
            elif token.type == "JOINT":
                node = BVHNode()
                node.assign_parent(layer_stack[-1])
                layer_stack[-1].add_child(node)
                node.parse_joint_name(bvh_lexer)
                layer_stack.append(node)
                self.node_list.append(node)
            elif token.type == "ENDJOINT":
                end_joint_encounter = True
                node = BVHNode()
                node.assign_parent(layer_stack[-1])
                layer_stack.append(node)
            elif token.type == "LPAREN":
                pass
            elif token.type == "RPAREN":
                if end_joint_encounter == True:
                    print(layer_stack[-1].parent.rest_head_world)
                    print(layer_stack[-1].rest_head_local)
                    print(layer_stack[-1].parent.rest_head_local)
                    layer_stack[-1].parent.rest_tail_world = layer_stack[-1].parent.rest_head_world + layer_stack[-1].rest_head_local
                    layer_stack[-1].parent.rest_tail_local = layer_stack[-1].parent.rest_head_local + layer_stack[-1].rest_head_local
                    end_joint_encounter = False
                layer_stack.pop()
            elif token.type == "MOTION":
                motion = Motion()
                self.motion_list.append(motion)
                motion.parse_motion(bvh_lexer, self.joint_parameter_amount)
            else:
                print("Unidentified token: ", token)

        # Now set the tip of each bvh_node
        for bvh_node in self.node_list:
            print(bvh_node.rest_tail_world, bvh_node.children)
            if bvh_node.rest_tail_world == None:
                if len(bvh_node.children) == 0:
                    # could just fail here, but rare BVH files have childless nodes
                    bvh_node.rest_tail_world = Vector(bvh_node.rest_head_world)
                    bvh_node.rest_tail_local = Vector(bvh_node.rest_head_local)
                elif len(bvh_node.children) == 1:
                    bvh_node.rest_tail_world = Vector(bvh_node.children[0].rest_head_world)
                    bvh_node.rest_tail_local = bvh_node.rest_head_local + bvh_node.children[0].rest_head_local
                else:
                    # allow this, see above
                    # if not bvh_node.children:
                    #	raise Exception("bvh node has no end and no children. bad file")

                    # Removed temp for now
                    rest_tail_world = Vector((0.0, 0.0, 0.0))
                    rest_tail_local = Vector((0.0, 0.0, 0.0))
                    for bvh_node_child in bvh_node.children:
                        rest_tail_world += bvh_node_child.rest_head_world
                        rest_tail_local += bvh_node_child.rest_head_local

                    bvh_node.rest_tail_world = rest_tail_world * (1.0 / len(bvh_node.children))
                    bvh_node.rest_tail_local = rest_tail_local * (1.0 / len(bvh_node.children))

            # Make sure tail isn't the same location as the head.
            if (bvh_node.rest_tail_local - bvh_node.rest_head_local).length <= 0.001:
                print("\tzero length node found:", bvh_node.joint_name)
                bvh_node.rest_tail_local.y = bvh_node.rest_tail_local.y + 1.0 / 10
                bvh_node.rest_tail_world.y = bvh_node.rest_tail_world.y + 1.0 / 10

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


def bvh_node_dict2armature(
        context,
        bvh_name,
        bvh_parser,
        frame_start=1
):
    if frame_start < 1:
        frame_start = 1

    # Add the new armature,
    scene = context.scene
    for obj in scene.objects:
        obj.select_set(False)

    arm_data = bpy.data.armatures.new(bvh_name)
    arm_ob = bpy.data.objects.new(bvh_name, arm_data)

    context.collection.objects.link(arm_ob)

    arm_ob.select_set(True)
    context.view_layer.objects.active = arm_ob

    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)

    bvh_nodes_list = bvh_parser.node_list

    # XXX, annoying, remove bone.
    while arm_data.edit_bones:
        arm_ob.edit_bones.remove(arm_data.edit_bones[-1])

    for bvh_node in bvh_nodes_list:
        # New editbone
        bone = bvh_node.temp = arm_data.edit_bones.new(bvh_node.joint_name)

        bone.head = bvh_node.rest_head_world
        bone.tail = bvh_node.rest_tail_world

    for bvh_node in bvh_nodes_list:
        if bvh_node.parent:
            # bvh_node.temp is the Editbone

            # Set the bone parent
            bvh_node.temp.parent = bvh_node.parent.temp

    # Replace the editbone with the editbone name,
    # to avoid memory errors accessing the editbone outside editmode
    for bvh_node in bvh_nodes_list:
        bvh_node.temp = bvh_node.temp.name

    # Now Apply the animation to the armature

    # Get armature animation data
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)

    pose = arm_ob.pose
    pose_bones = pose.bones

    for pose_bone in pose_bones:
        pose_bone.rotation_mode = "XYZ"

    context.view_layer.update()

    # arm_ob.animation_data_create()
    # action = bpy.data.actions.new(name=bvh_name)
    # arm_ob.animation_data.action = action
    #
    # # Replace the bvh_node.temp (currently an editbone)
    # # With a tuple  (pose_bone, armature_bone, bone_rest_matrix, bone_rest_matrix_inv)
    # num_frame = 0
    # for bvh_node in bvh_nodes_list:
    # 	bone_name = bvh_node.temp  # may not be the same name as the bvh_node, could have been shortened.
    # 	pose_bone = pose_bones[bone_name]
    # 	rest_bone = arm_data.bones[bone_name]
    # 	bone_rest_matrix = rest_bone.matrix_local.to_3x3()
    #
    # 	bone_rest_matrix_inv = Matrix(bone_rest_matrix)
    # 	bone_rest_matrix_inv.invert()
    #
    # 	bone_rest_matrix_inv.resize_4x4()
    # 	bone_rest_matrix.resize_4x4()
    # 	bvh_node.temp = (pose_bone, bone, bone_rest_matrix, bone_rest_matrix_inv)
    #
    # 	if 0 == num_frame:
    # 		num_frame = len(bvh_node.anim_data)
    #
    # # Choose to skip some frames at the beginning. Frame 0 is the rest pose
    # # used internally by this importer. Frame 1, by convention, is also often
    # # the rest pose of the skeleton exported by the motion capture system.
    # skip_frame = 1
    # if num_frame > skip_frame:
    # 	num_frame = num_frame - skip_frame
    #
    # # Create a shared time axis for all animation curves.
    # time = [float(frame_start)] * num_frame
    # if use_fps_scale:
    # 	dt = scene.render.fps * bvh_frame_time
    # 	for frame_i in range(1, num_frame):
    # 		time[frame_i] += float(frame_i) * dt
    # else:
    # 	for frame_i in range(1, num_frame):
    # 		time[frame_i] += float(frame_i)
    #
    # # print("bvh_frame_time = %f, dt = %f, num_frame = %d"
    # #      % (bvh_frame_time, dt, num_frame]))
    #
    # for i, bvh_node in enumerate(bvh_nodes_list):
    # 	pose_bone, bone, bone_rest_matrix, bone_rest_matrix_inv = bvh_node.temp
    #
    # 	if bvh_node.has_loc:
    # 		# Not sure if there is a way to query this or access it in the
    # 		# PoseBone structure.
    # 		data_path = 'pose.bones["%s"].location' % pose_bone.name
    #
    # 		location = [(0.0, 0.0, 0.0)] * num_frame
    # 		for frame_i in range(num_frame):
    # 			bvh_loc = bvh_node.anim_data[frame_i + skip_frame][:3]
    #
    # 			bone_translate_matrix = Matrix.Translation(
    # 				Vector(bvh_loc) - bvh_node.rest_head_local)
    # 			location[frame_i] = (bone_rest_matrix_inv @
    # 								 bone_translate_matrix).to_translation()
    #
    # 		# For each location x, y, z.
    # 		for axis_i in range(3):
    # 			curve = action.fcurves.new(data_path=data_path, index=axis_i)
    # 			keyframe_points = curve.keyframe_points
    # 			keyframe_points.add(num_frame)
    #
    # 			for frame_i in range(num_frame):
    # 				keyframe_points[frame_i].co = (
    # 					time[frame_i],
    # 					location[frame_i][axis_i],
    # 				)
    #
    # 	if bvh_node.has_rot:
    # 		data_path = None
    # 		rotate = None
    #
    # 		if 'QUATERNION' == rotate_mode:
    # 			rotate = [(1.0, 0.0, 0.0, 0.0)] * num_frame
    # 			data_path = ('pose.bones["%s"].rotation_quaternion'
    # 						 % pose_bone.name)
    # 		else:
    # 			rotate = [(0.0, 0.0, 0.0)] * num_frame
    # 			data_path = ('pose.bones["%s"].rotation_euler' %
    # 						 pose_bone.name)
    #
    # 		prev_euler = Euler((0.0, 0.0, 0.0))
    # 		for frame_i in range(num_frame):
    # 			bvh_rot = bvh_node.anim_data[frame_i + skip_frame][3:]
    #
    # 			# apply rotation order and convert to XYZ
    # 			# note that the rot_order_str is reversed.
    # 			euler = Euler(bvh_rot, bvh_node.rot_order_str[::-1])
    # 			bone_rotation_matrix = euler.to_matrix().to_4x4()
    # 			bone_rotation_matrix = (
    # 				bone_rest_matrix_inv @
    # 				bone_rotation_matrix @
    # 				bone_rest_matrix
    # 			)
    #
    # 			if len(rotate[frame_i]) == 4:
    # 				rotate[frame_i] = bone_rotation_matrix.to_quaternion()
    # 			else:
    # 				rotate[frame_i] = bone_rotation_matrix.to_euler(
    # 					pose_bone.rotation_mode, prev_euler)
    # 				prev_euler = rotate[frame_i]
    #
    # 		# For each euler angle x, y, z (or quaternion w, x, y, z).
    # 		for axis_i in range(len(rotate[0])):
    # 			curve = action.fcurves.new(data_path=data_path, index=axis_i)
    # 			keyframe_points = curve.keyframe_points
    # 			keyframe_points.add(num_frame)
    #
    # 			for frame_i in range(num_frame):
    # 				keyframe_points[frame_i].co = (
    # 					time[frame_i],
    # 					rotate[frame_i][axis_i],
    # 				)
    #
    # for cu in action.fcurves:
    # 	if IMPORT_LOOP:
    # 		pass  # 2.5 doenst have cyclic now?
    #
    # 	for bez in cu.keyframe_points:
    # 		bez.interpolation = 'LINEAR'
    #
    # # finally apply matrix
    # arm_ob.matrix_world = global_matrix
    # bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

    return arm_ob


def load(context, bvh_filepath):
    parser = BVHParser(bvh_filepath)

    scene = context.scene
    frame_orig = scene.frame_current
    bvh_name = bpy.path.display_name_from_filepath(bvh_filepath)
    bvh_node_dict2armature(context, bvh_name, parser)
    context.scene.frame_set(frame_orig)
    return {'FINISHED'}


if __name__ == '__main__':
    parser = BVHParser("../bvh_example/01_01.bvh")
