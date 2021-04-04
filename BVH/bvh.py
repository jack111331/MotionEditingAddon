from . import bvh_lex
import bpy
from math import radians, degrees
from mathutils import Vector, Matrix, Euler
import os


class BVHNode:
    channel_layout = ['XPOSITION', 'YPOSITION',
                      'ZPOSITION', 'XROTATION', 'YROTATION', 'ZROTATION']

    def __init__(self):
        self.joint_name = ""
        self.channels = 0
        self.channel_layout_assign = [-1, -1, -1, -1, -1, -1]
        self.rot_order_str = ""
        self.in_motion_start_idx = 0
        self.rest_head_local = Vector((0.0, 0.0, 0.0))
        self.rest_head_world = Vector((0.0, 0.0, 0.0))
        self.rest_tail_local = None
        self.rest_tail_world = None
        self.model_matrix = Matrix.Identity(4)
        self.parent = None
        self.children = []
        self.has_loc = False
        self.has_rot = False
        self.edit_bone = None
        self.in_edit_bone_name = ""
        self.in_list_index = 0
        self.temp = None

    def clone(self):
        new_node = BVHNode()
        new_node.joint_name = self.joint_name
        new_node.channels = self.channels
        new_node.channel_layout_assign = self.channel_layout_assign
        new_node.rot_order_str = self.rot_order_str
        new_node.in_motion_start_idx = self.in_motion_start_idx
        new_node.rest_head_local = self.rest_head_local.copy()
        new_node.rest_head_world = self.rest_head_world.copy()
        new_node.rest_tail_local = self.rest_tail_local.copy()
        new_node.rest_tail_world = self.rest_tail_world.copy()
        new_node.model_matrix = self.model_matrix.copy()
        new_node.parent = None
        if self.parent != None:
            new_node.parent = self.parent.joint_name
        new_node.children = [node.joint_name for node in self.children]
        new_node.has_loc = self.has_loc
        new_node.has_rot = self.has_rot
        new_node.edit_bone = self.edit_bone
        new_node.in_edit_bone_name = self.in_edit_bone_name
        new_node.in_list_index = self.in_list_index
        new_node.temp = self.temp
        return new_node

    @staticmethod
    def clone_list(node_list):
        new_node_dict = {}
        for i in range(len(node_list)):
            new_node_dict[node_list[i].joint_name] = node_list[i].clone()
        for node in new_node_dict.values():
            if node.parent != None:
                node.parent = new_node_dict[node.parent]
            for i in range(len(node.children)):
                node.children[i] = new_node_dict[node.children[i]]

        return list(new_node_dict.values())

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
            if type == "XROTATION":
                self.rot_order_str += "X"
            elif type == "YROTATION":
                self.rot_order_str += "Y"
            elif type == "ZROTATION":
                self.rot_order_str += "Z"
            for j in range(len(BVHNode.channel_layout)):
                if type == BVHNode.channel_layout[j]:
                    self.channel_layout_assign[j] = i
                    break
        self.in_motion_start_idx = joint_parameter_amount
        self.has_loc = self.channel_layout_assign[0] != -1 or self.channel_layout_assign[1] != -1 or \
                       self.channel_layout_assign[2] != -1
        self.has_rot = self.channel_layout_assign[3] != -1 or self.channel_layout_assign[4] != -1 or \
                       self.channel_layout_assign[5] != -1
        return self.channels

    def compare_to(self, other_node):
        if other_node.joint_name != self.joint_name:
            return False
        if other_node.channels != self.channels:
            return False
        if other_node.channel_layout_assign != self.channel_layout_assign:
            return False
        if other_node.in_motion_start_idx != self.in_motion_start_idx:
            return False

        if (other_node.rest_head_local - self.rest_head_local) >= 1e-6:
            return False
        if (other_node.rest_tail_local - self.rest_tail_local) >= 1e-6:
            return False
        if self.parent != None and other_node.parent.joint_name != self.parent.joint_name:
            return False
        my_children_name_list = [children.joint_name for children in self.children]
        for children in other_node.children:
            if children.joint_name not in my_children_name_list:
                return False
        return True


class AggregateFrame:
    def __init__(self):
        self.node_list = None
        self.frame = None

    def assign_node_list(self, node_list):
        self.node_list = node_list

    def assign_frame(self, frame):
        self.frame = frame

    def get_loc(self, bone_index):
        channel = self.frame.motion_parameter_list[
                  self.node_list[bone_index].in_motion_start_idx:self.node_list[bone_index].in_motion_start_idx +
                                                                 self.node_list[bone_index].channels]
        x, y, z = 0.0, 0.0, 0.0
        if self.node_list[bone_index].channel_layout_assign[0] != -1:
            x = channel[self.node_list[bone_index].channel_layout_assign[0]]
        if self.node_list[bone_index].channel_layout_assign[1] != -1:
            y = channel[self.node_list[bone_index].channel_layout_assign[1]]
        if self.node_list[bone_index].channel_layout_assign[2] != -1:
            z = channel[self.node_list[bone_index].channel_layout_assign[2]]

        return [x, y, z]

    def get_rot(self, bone_index):
        channel = self.frame.motion_parameter_list[
                  self.node_list[bone_index].in_motion_start_idx:self.node_list[bone_index].in_motion_start_idx +
                                                                 self.node_list[bone_index].channels]
        x, y, z = 0.0, 0.0, 0.0
        if self.node_list[bone_index].channel_layout_assign[3] != -1:
            x = channel[self.node_list[bone_index].channel_layout_assign[3]]
        if self.node_list[bone_index].channel_layout_assign[4] != -1:
            y = channel[self.node_list[bone_index].channel_layout_assign[4]]
        if self.node_list[bone_index].channel_layout_assign[5] != -1:
            z = channel[self.node_list[bone_index].channel_layout_assign[5]]

        return [radians(x), radians(y), radians(z)]

    def to_channel(self, bone_index, loc, rot):
        channel = [0] * self.node_list[bone_index].channels
        for i in range(3):
            if self.node_list[bone_index].channel_layout_assign[i] != -1:
                channel[self.node_list[bone_index].channel_layout_assign[i]] = loc[i]
            if self.node_list[bone_index].channel_layout_assign[i + 3] != -1:
                channel[self.node_list[bone_index].channel_layout_assign[i + 3]] = rot[i]
        return channel


    def added_offset_to_channel(self, bone_index, loc, rot):
        channel = [0] * self.node_list[bone_index].channels
        in_motion_start_idx = self.node_list[bone_index].in_motion_start_idx
        for i in range(3):
            if self.node_list[bone_index].channel_layout_assign[i] != -1:
                channel[self.node_list[bone_index].channel_layout_assign[i]] = self.frame.motion_parameter_list[in_motion_start_idx+self.node_list[bone_index].channel_layout_assign[i]] + loc[i]
            if self.node_list[bone_index].channel_layout_assign[i + 3] != -1:
                channel[self.node_list[bone_index].channel_layout_assign[i + 3]] = self.frame.motion_parameter_list[in_motion_start_idx+self.node_list[bone_index].channel_layout_assign[i+3]] + rot[i]
        return channel


class Frame:
    def __init__(self):
        self.motion_parameter_list = []

    def parse_frame(self, bvh_lex, joint_parameter_amount):
        for i in range(joint_parameter_amount):
            self.motion_parameter_list.append(bvh_lex.token().value)

    def clone(self):
        new_frame = Frame()
        new_frame.motion_parameter_list = self.motion_parameter_list[:]
        return new_frame

    @staticmethod
    def clone_list(frame_list):
        new_frame_list = []
        for frame in frame_list:
            new_frame_list.append(frame.clone())
        return new_frame_list


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

    def generate_root_pos_and_orientation(self, node_list, root_transform_matrix_list=None):
        aggregate_frame = AggregateFrame()
        aggregate_frame.assign_node_list(node_list)

        root_pos_and_orientation_list = []
        root_node = node_list[0].clone()
        num_frame = len(self.frame_list)
        for frame_i in range(num_frame):
            root_transform_matrix = Matrix.Identity(4)
            if root_transform_matrix_list != None:
                root_transform_matrix = root_transform_matrix_list[frame_i]

            aggregate_frame.assign_frame(self.frame_list[frame_i])
            offset_matrix = Matrix.Translation(root_node.rest_head_local)
            translation_matrix = Matrix.Translation(aggregate_frame.get_loc(0))
            euler = Euler(aggregate_frame.get_rot(0), root_node.rot_order_str[::-1])
            rotation_matrix = euler.to_matrix().to_4x4()

            root_node.model_matrix = root_transform_matrix @ offset_matrix @ translation_matrix @ rotation_matrix

            root_node.rest_head_world = root_node.model_matrix @ Vector((0.0, 0.0, 0.0))

            orientation_euler_angle = root_node.model_matrix.to_euler(root_node.rot_order_str[::-1])
            orientation_euler_angle = [degrees(angle) for angle in orientation_euler_angle]
            root_pos_and_orientation_list.append(list(root_node.rest_head_world[:]) + orientation_euler_angle)
        return root_pos_and_orientation_list

    def generate_all_node_pos_and_orientation(self, bvh_parser, root_transform_matrix_list):
        new_motion = Motion()
        new_motion.frame_time = self.frame_time
        new_motion.frame_amount = self.frame_amount
        new_node_list = BVHNode.clone_list(bvh_parser.node_list)
        num_frame = len(self.frame_list)

        aggregate_frame = AggregateFrame()
        aggregate_frame.assign_node_list(new_node_list)

        for frame_i in range(num_frame):
            new_frame = Frame()
            new_frame.motion_parameter_list = [0.0] * bvh_parser.joint_parameter_amount
            root_transform_matrix = root_transform_matrix_list[frame_i]
            aggregate_frame.assign_frame(self.frame_list[frame_i])
            generate_queue = [new_node_list[0]]
            while len(generate_queue) != 0:
                top = generate_queue[0]
                generate_queue.pop(0)
                for children in top.children:
                    generate_queue.append(children)

                offset_matrix = Matrix.Translation(top.rest_head_local)
                translation_matrix = Matrix.Translation(aggregate_frame.get_loc(top.in_list_index))
                # radians
                rot = aggregate_frame.get_rot(top.in_list_index)
                euler = Euler(rot, top.rot_order_str[::-1])
                rotation_matrix = euler.to_matrix().to_4x4()

                if top.in_list_index == 0:
                    top.model_matrix = root_transform_matrix @ offset_matrix @ translation_matrix @ rotation_matrix
                else:
                    top.model_matrix = top.parent.model_matrix @ offset_matrix @ translation_matrix @ rotation_matrix

                top.rest_head_world = top.model_matrix @ Vector((0.0, 0.0, 0.0))
                top.rest_tail_world = top.model_matrix @ Vector(top.rest_tail_local - top.rest_head_local)

                orientation_euler_angle = (root_transform_matrix @ rotation_matrix).to_euler(top.rot_order_str[::-1])
                orientation_euler_angle = [degrees(angle) for angle in orientation_euler_angle]
                new_frame.motion_parameter_list[
                top.in_motion_start_idx:top.in_motion_start_idx + top.channels] = aggregate_frame.to_channel(
                    top.in_list_index, list(top.rest_head_world[:]), orientation_euler_angle)

            new_motion.frame_list.append(new_frame)
        return new_motion

    def apply_motion_on_armature(self, context, bvh_nodes_list, arm_ob, frame_start, global_matrix, initialize=False):
        # Should select object first.....
        arm_ob.select_set(True)
        context.view_layer.objects.active = arm_ob
        context.view_layer.update()

        pose = arm_ob.pose
        pose_bones = pose.bones
        action = arm_ob.animation_data.action
        arm_data = arm_ob.data

        if initialize == False:
            # Recover to original pose coordinate system
            arm_ob.matrix_world = global_matrix.inverted()
            bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

        # Replace the bvh_node.temp
        # With a tuple  (pose_bone, armature_bone, bone_rest_matrix, bone_rest_matrix_inv)
        num_frame = 0
        for bvh_node in bvh_nodes_list:
            bone_name = bvh_node.in_edit_bone_name
            pose_bone = pose_bones[bone_name]
            rest_bone = arm_data.bones[bone_name]
            bone_rest_matrix = rest_bone.matrix_local.to_3x3()

            bone_rest_matrix_inv = Matrix(bone_rest_matrix)
            bone_rest_matrix_inv.invert()

            bone_rest_matrix_inv.resize_4x4()
            bone_rest_matrix.resize_4x4()
            bvh_node.temp = (pose_bone, bone_rest_matrix, bone_rest_matrix_inv)

            if 0 == num_frame:
                num_frame = self.frame_amount

        # Choose to skip some frames at the beginning. Frame 0 is the rest pose
        # used internally by this importer. Frame 1, by convention, is also often
        # the rest pose of the skeleton exported by the motion capture system.
        skip_frame = 0
        if num_frame > skip_frame:
            num_frame = num_frame - skip_frame

        # Create a shared time axis for all animation curves.
        time = [float(frame_start)] * num_frame
        for frame_i in range(1, num_frame):
            time[frame_i] += float(frame_i)

        # print("bvh_frame_time = %f, dt = %f, num_frame = %d"
        #      % (bvh_frame_time, dt, num_frame]))

        aggregate_frame = AggregateFrame()
        aggregate_frame.assign_node_list(bvh_nodes_list)
        for i, bvh_node in enumerate(bvh_nodes_list):
            pose_bone, bone_rest_matrix, bone_rest_matrix_inv = bvh_node.temp

            if bvh_node.has_loc:
                # Not sure if there is a way to query this or access it in the
                # PoseBone structure.
                data_path = 'pose.bones["%s"].location' % pose_bone.name

                location = [(0.0, 0.0, 0.0)] * num_frame
                for frame_i in range(num_frame):
                    aggregate_frame.assign_frame(self.frame_list[frame_i])
                    bvh_loc = aggregate_frame.get_loc(i)

                    bone_translate_matrix = Matrix.Translation(
                        Vector(bvh_loc) - bvh_node.rest_head_local)
                    location[frame_i] = (bone_rest_matrix_inv @
                                         bone_translate_matrix).to_translation()

                # For each location x, y, z.
                for axis_i in range(3):
                    curve = action.fcurves.find(data_path=data_path, index=axis_i)
                    if curve != None:
                        action.fcurves.remove(curve)
                    curve = action.fcurves.new(data_path=data_path, index=axis_i)

                    keyframe_points = curve.keyframe_points
                    keyframe_points.add(num_frame)

                    for frame_i in range(num_frame):
                        keyframe_points[frame_i].co = (
                            time[frame_i],
                            location[frame_i][axis_i],
                        )

            if bvh_node.has_rot:
                rotate = [(0.0, 0.0, 0.0)] * num_frame
                data_path = ('pose.bones["%s"].rotation_euler' %
                             pose_bone.name)

                prev_euler = Euler((0.0, 0.0, 0.0))
                for frame_i in range(num_frame):
                    aggregate_frame.assign_frame(self.frame_list[frame_i])
                    bvh_rot = aggregate_frame.get_rot(i)

                    # apply rotation order and convert to XYZ
                    # note that the rot_order_str is reversed.
                    euler = Euler(bvh_rot, bvh_node.rot_order_str[::-1])

                    bone_rotation_matrix = euler.to_matrix().to_4x4()
                    bone_rotation_matrix = (
                            bone_rest_matrix_inv @
                            bone_rotation_matrix @
                            bone_rest_matrix
                    )

                    rotate[frame_i] = bone_rotation_matrix.to_euler(
                        pose_bone.rotation_mode, prev_euler)
                    prev_euler = rotate[frame_i]

                # For each euler angle x, y, z (or quaternion w, x, y, z).
                for axis_i in range(len(rotate[0])):
                    curve = action.fcurves.find(data_path=data_path, index=axis_i)
                    if curve != None:
                        action.fcurves.remove(curve)
                    curve = action.fcurves.new(data_path=data_path, index=axis_i)
                    keyframe_points = curve.keyframe_points
                    keyframe_points.add(num_frame)

                    for frame_i in range(num_frame):
                        keyframe_points[frame_i].co = (
                            time[frame_i],
                            rotate[frame_i][axis_i],
                        )

        for cu in action.fcurves:
            for bez in cu.keyframe_points:
                bez.interpolation = 'LINEAR'

        # finally apply matrix
        arm_ob.matrix_world = global_matrix
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

    def clone(self):
        new_motion = Motion()
        new_motion.frame_list = Frame.clone_list(self.frame_list)
        new_motion.frame_amount = self.frame_amount
        new_motion.frame_time = self.frame_time
        return new_motion

    def concatenate(self, bvh_parser, concatenate_motion):
        concatenate_start_frame_idx = self.frame_amount
        aggregate_frame = AggregateFrame()
        aggregate_frame.assign_node_list(bvh_parser.node_list)
        aggregate_frame.assign_frame(self.frame_list[-1])
        last_loc = aggregate_frame.get_loc(0)
        last_rot = aggregate_frame.get_rot(0)
        aggregate_frame.assign_frame(concatenate_motion.frame_list[0])
        new_loc = aggregate_frame.get_loc(0)
        new_rot = aggregate_frame.get_rot(0)

        # drag concatenate motion to current motion
        offset_loc = [last_loc[i] - new_loc[i] for i in range(len(new_loc))]
        offset_rot = [last_rot[i] - new_rot[i] for i in range(len(new_loc))]

        for frame_i in range(len(concatenate_motion.frame_list)):
            new_frame = concatenate_motion.frame_list[frame_i].clone()
            aggregate_frame.assign_frame(concatenate_motion.frame_list[frame_i])
            for i in range(len(bvh_parser.node_list)):
                in_motion_start_idx = bvh_parser.node_list[i].in_motion_start_idx
                channels = bvh_parser.node_list[i].channels
                new_frame.motion_parameter_list[in_motion_start_idx:in_motion_start_idx + channels] = aggregate_frame.added_offset_to_channel(
                    i, offset_loc, offset_rot)
            self.frame_list.append(new_frame)

        self.frame_amount = len(self.frame_list)


class BVHParser:

    def __init__(self, bvh_filename=""):
        self.node_list = []
        self.joint_parameter_amount = 0
        self.motion = None
        self.name = os.path.splitext(os.path.basename(bvh_filename))[0]

        if len(bvh_filename) != 0:
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
                    node.in_list_index = len(self.node_list)
                    layer_stack.append(node)
                    self.node_list.append(node)
                elif token.type == "OFFSET":
                    layer_stack[-1].parse_offset(bvh_lexer)
                elif token.type == "CHANNELS":
                    joint_parameter_amount = layer_stack[-1].parse_channel_layout(bvh_lexer,
                                                                                  self.joint_parameter_amount)
                    self.joint_parameter_amount += joint_parameter_amount
                elif token.type == "JOINT":
                    node = BVHNode()
                    node.assign_parent(layer_stack[-1])
                    layer_stack[-1].add_child(node)
                    node.parse_joint_name(bvh_lexer)
                    node.in_list_index = len(self.node_list)
                    layer_stack.append(node)
                    self.node_list.append(node)
                elif token.type == "ENDJOINT":
                    end_joint_encounter = True
                    node = BVHNode()
                    node.assign_parent(layer_stack[-1])
                    node.joint_name = "Dummy"
                    layer_stack.append(node)
                elif token.type == "LPAREN":
                    pass
                elif token.type == "RPAREN":
                    if end_joint_encounter == True:
                        layer_stack[-1].parent.rest_tail_world = layer_stack[-1].parent.rest_head_world + layer_stack[
                            -1].rest_head_local
                        layer_stack[-1].parent.rest_tail_local = layer_stack[-1].parent.rest_head_local + layer_stack[
                            -1].rest_head_local
                        end_joint_encounter = False
                    layer_stack.pop()
                elif token.type == "MOTION":
                    motion = Motion()
                    self.motion = motion
                    motion.parse_motion(bvh_lexer, self.joint_parameter_amount)
                else:
                    print("Unidentified token: ", token)

            # Now set the tip of each bvh_node
            for bvh_node in self.node_list:
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
                self.motion = motion
                motion.parse_motion(bvh_lexer, self.node_list)
            else:
                print("Unidentified token")

    def compare_to(self, other_parser):
        if self.joint_parameter_amount != other_parser.joint_parameter_amount:
            return False
        if len(self.node_list) != len(other_parser.node_list):
            return False
        for i in range(len(self.node_list)):
            # compare node
            if self.node_list[i].compare_to(other_parser.node_list[i]) == False:
                return False
        return True

    def clone(self):
        new_parser = BVHParser()
        new_parser.node_list = BVHNode.clone_list(self.node_list)
        new_parser.joint_parameter_amount = self.joint_parameter_amount
        new_parser.motion = self.motion.clone()
        new_parser.name = "" + self.name
        return new_parser


def bvh_node_dict2armature(
        context,
        bvh_name,
        bvh_parser,
        frame_start=1,
        global_matrix=None
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

    # Get the average bone length for zero length bones, we may not use this.
    average_bone_length = 0.0
    nonzero_count = 0
    for bvh_node in bvh_nodes_list:
        l = (bvh_node.rest_head_local - bvh_node.rest_tail_local).length
        if l:
            average_bone_length += l
            nonzero_count += 1

    # Very rare cases all bones could be zero length???
    if not average_bone_length:
        average_bone_length = 0.1
    else:
        # Normal operation
        average_bone_length = average_bone_length / nonzero_count

    # XXX, annoying, remove bone.
    while arm_data.edit_bones:
        arm_ob.edit_bones.remove(arm_data.edit_bones[-1])

    ZERO_AREA_BONES = []
    for bvh_node in bvh_nodes_list:
        # New editbone
        bone = bvh_node.edit_bone = arm_data.edit_bones.new(bvh_node.joint_name)

        bone.head = bvh_node.rest_head_world
        bone.tail = bvh_node.rest_tail_world

        # Zero Length Bones! (an exceptional case)
        if (bone.head - bone.tail).length < 0.001:
            print("\tzero length bone found:", bone.name)
            if bvh_node.parent:
                ofs = bvh_node.parent.rest_head_local - bvh_node.parent.rest_tail_local
                if ofs.length:  # is our parent zero length also?? unlikely
                    bone.tail = bone.tail - ofs
                else:
                    bone.tail.y = bone.tail.y + average_bone_length
            else:
                bone.tail.y = bone.tail.y + average_bone_length

            ZERO_AREA_BONES.append(bone.name)

    for bvh_node in bvh_nodes_list:
        if bvh_node.parent:
            # Set the bone parent
            bvh_node.edit_bone.parent = bvh_node.parent.edit_bone

            # Set the connection state
            if (
                    (not bvh_node.has_loc) and
                    (bvh_node.parent.edit_bone.name not in ZERO_AREA_BONES) and
                    (bvh_node.parent.rest_tail_local == bvh_node.rest_head_local)
            ):
                bvh_node.edit_bone.use_connect = True

    # Replace the editbone with the editbone name,
    # to avoid memory errors accessing the editbone outside editmode
    for bvh_node in bvh_nodes_list:
        bvh_node.in_edit_bone_name = bvh_node.edit_bone.name

    # Now Apply the animation to the armature

    # Get armature animation data
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)

    pose = arm_ob.pose
    pose_bones = pose.bones

    for bvh_node in bvh_nodes_list:
        bone_name = bvh_node.in_edit_bone_name  # may not be the same name as the bvh_node, could have been shortened.
        pose_bone = pose_bones[bone_name]
        pose_bone.rotation_mode = bvh_node.rot_order_str

    context.view_layer.update()

    arm_ob.animation_data_create()
    action = bpy.data.actions.new(name=bvh_name)
    arm_ob.animation_data.action = action

    bvh_parser.motion.apply_motion_on_armature(context, bvh_parser.node_list, arm_ob, frame_start,
                                               global_matrix, True)

    return arm_ob


def load(context, filepath, global_matrix=None):
    parser = BVHParser(filepath)

    scene = context.scene
    frame_orig = scene.frame_current
    bvh_name = bpy.path.display_name_from_filepath(filepath)
    bvh_node_dict2armature(context, bvh_name, parser, global_matrix=global_matrix)
    context.scene.frame_set(frame_orig)
    return parser


if __name__ == '__main__':
    parser = BVHParser("../bvh_example/01_01.bvh")
