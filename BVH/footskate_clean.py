import bpy
from mathutils import geometry, Vector, Matrix, Euler, Quaternion
from math import acos
from . import motion_editing
from . import bvh


def solve_inverse_kinematics(joint_pos, joint_rot, target, use_constraint, constraint, iterations=10, epsilon=0.0001):
    total_length = 0

    bone_length = [0] * (len(joint_pos) - 1)

    for i in range(len(joint_pos) - 1):
        direction = (joint_pos[i] - joint_pos[i + 1])
        bone_length[i] = direction.magnitude
        total_length += bone_length[i]

    if (target - joint_pos[-1]).magnitude > total_length:
        # stretch bone if hip plus total length can't achieve ground
        for i in range(len(joint_pos) - 2, -1, -1):
            direction = (target - joint_pos[i + 1]).normalized()
            joint_pos[i] = joint_pos[i + 1] + direction * bone_length[i]

    else:
        # try to solve the joints' pos and rot so that we can achieve the joint on ground
        last_pos = joint_pos[0].copy()
        # iterate backward and forward
        for k in range(iterations):
            joint_pos[0] = target.copy()
            for i in range(1, len(joint_pos) - 1):
                direction = (joint_pos[i] - joint_pos[i - 1]).normalized()
                joint_pos[i] = joint_pos[i - 1] + direction * bone_length[i - 1]

            for i in range(len(joint_pos) - 2, -1, -1):
                direction = (joint_pos[i] - joint_pos[i + 1]).normalized()
                joint_pos[i] = joint_pos[i + 1] + direction * bone_length[i]

            if (joint_pos[0] - last_pos).magnitude < epsilon:
                break

            last_pos = joint_pos[0].copy()

    if use_constraint:
        for i in range(1, len(joint_pos) - 1):
            normal = (joint_pos[i + 1] - joint_pos[i - 1]).normalized()

            point = joint_pos[i - 1]

            projection_pole = geometry.intersect_line_plane(constraint, constraint + normal, point, normal)
            projection_bone = geometry.intersect_line_plane(joint_pos[i], joint_pos[i] + normal, point, normal)

            Va = (projection_bone - joint_pos[i - 1])
            Vb = (projection_pole - joint_pos[i - 1])

            dotV = max(-1, min(1, Va.normalized().dot(Vb.normalized())))

            angle = acos(dotV)
            cross = Va.cross(Vb)

            if normal.dot(cross) < 0:
                angle = -angle

            joint_pos[i] = Quaternion(normal, angle) @ (joint_pos[i] - joint_pos[i - 1]) + joint_pos[i - 1]

    for i in range(1, len(joint_pos)):
        bone_direction = (joint_pos[i - 1] - joint_pos[i]).normalized()
        joint_rot[i] = Vector((0, 0, -1)).rotation_difference(bone_direction)

    return joint_pos, joint_rot


class FootSkateCleanOperator(bpy.types.Operator):
    bl_idname = "footskate_clean.keyframe"
    bl_label = "Clean one motion's footskate"
    bl_description = "select one motion collection and clean the foot skate"

    selected_animation = None
    ground_height = 0

    def load_joint(self, context):
        enum_items = []

        motion_path_animation = FootSkateCleanOperator.selected_animation
        if motion_path_animation != None:
            for node in motion_path_animation.bvh_parser.node_list:
                # real value, display name, discription
                enum_items.append((str(node.in_list_index), node.joint_name, node.joint_name, "", node.in_list_index))

        if len(enum_items) == 0:
            enum_items.append(("None", "None", "None"))

        return enum_items

    left_foot = bpy.props.EnumProperty(name="left_foot", items=load_joint)
    right_foot = bpy.props.EnumProperty(name="right_foot", items=load_joint)

    @classmethod
    def poll(cls, context):
        collection_name = context.scene.footskate_clean_select_collection_name

        motion_path_animation = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name)

        return motion_path_animation != None

    def invoke(self, context, event):
        wm = context.window_manager

        collection_name = bpy.context.scene.footskate_clean_select_collection_name
        motion_path_animation = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name)

        FootSkateCleanOperator.selected_animation = motion_path_animation

        return wm.invoke_props_dialog(self)

    def execute(self, context):

        def is_valid_foot_node(node):
            return node != None and node.parent != None and node.parent.parent != None and node.parent.parent.parent != None

        motion_path_animation = FootSkateCleanOperator.selected_animation
        if motion_path_animation != None:
            left_foot_node = motion_path_animation.bvh_parser.node_list[int(self.left_foot)]
            right_foot_node = motion_path_animation.bvh_parser.node_list[int(self.right_foot)]

            if is_valid_foot_node(left_foot_node) == False or is_valid_foot_node(right_foot_node) == False:
                self.report({"ERROR"}, "Illegal joint node!")
                return {"CANCELLED"}

            self.replace_animation(motion_path_animation, left_foot_node, right_foot_node)
            arm_ob = bpy.data.objects[motion_path_animation.bvh_parser.name]
            motion_path_animation.new_motion_data.apply_motion_on_armature(context,
                                                                             motion_path_animation.bvh_parser.node_list,
                                                                             arm_ob, 1)

        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.label(text="Select foot joints")

        row = layout.row()
        row.prop(self, "left_foot")

        row = layout.row()
        row.prop(self, "right_foot")

    def replace_animation(self, motion_path_animation, left_foot_node, right_foot_node):
        self.solve_foot_node(motion_path_animation, left_foot_node)
        self.solve_foot_node(motion_path_animation, right_foot_node)

    def solve_foot_node(self, animation, foot_node):
        knee_node = foot_node.parent
        hip_node = knee_node.parent

        nodes = (foot_node, knee_node, hip_node)

        target_node_dict = {node.in_list_index: i for i, node in enumerate(nodes)}
        world_loc_and_rot_list = animation.new_motion_data.generate_all_node_pos_and_orientation(
            animation.bvh_parser, target_node_dict)

        for frame_i in range(len(animation.new_motion_data.frame_list)):
            # Setup constraint ankle position on ground
            world_loc_and_rot = world_loc_and_rot_list[frame_i]
            joint_poses = [Vector(loc_and_rot[:3]) for loc_and_rot in world_loc_and_rot]
            joint_rots = [Euler(loc_and_rot[3:], nodes[i].rot_order_str[::-1]) for i, loc_and_rot in enumerate(world_loc_and_rot)]

            intersection = geometry.intersect_line_plane(joint_poses[0], joint_poses[0] + Vector((0, 0, 1)),
                                                         Vector((0, self.ground_height, 0)), Vector((0, 0, 1)))
            if intersection[2] > joint_poses[0][2]:
                print("Before pos:", joint_poses)
                print("Before rot:", joint_rots)
                new_joint_poses, new_joint_rots = solve_inverse_kinematics(joint_poses, joint_rots, intersection, True,
                                                                           joint_poses[1].copy(), iterations=15)
                print("After pos:", new_joint_poses)
                print("After rot:", new_joint_rots)

                # Apply new position and rotation to nodes in original animation
                for i in range(len(nodes)):
                    translation_matrix = Matrix.Translation(new_joint_poses[i])
                    rotation_matrix = new_joint_rots[i].to_matrix().to_4x4()
                    transform_matrix = translation_matrix @ rotation_matrix
                    nodes[i].model_matrix = transform_matrix

                    head_world_pos = nodes[i].model_matrix.to_translation()
                    # get bvh local euler angle
                    head_world_rot = nodes[i].model_matrix.to_euler(nodes[i].rot_order_str[::-1])
                    head_world_rot = [angle for angle in head_world_rot]

                    animation.new_motion_data.frame_list[frame_i].set_loc(animation.bvh_parser.node_list, nodes[i].in_list_index, head_world_pos)
                    animation.new_motion_data.frame_list[frame_i].set_rot(animation.bvh_parser.node_list, nodes[i].in_list_index, head_world_rot)

def draw(context, layout):
    row = layout.row()
    row.label(text="Per frame inverse kinematics")

    row = layout.row()
    row.prop_search(
        data=context.scene,
        property="footskate_clean_select_collection_name",
        search_data=bpy.data,
        search_property="collections",
        text="animation")

    row = layout.row()
    row.operator("footskate_clean.keyframe", text="Fix footskate")


def register():
    bpy.utils.register_class(FootSkateCleanOperator)

    bpy.types.Scene.footskate_clean_select_collection_name = bpy.props.StringProperty()


def unregister():
    bpy.utils.unregister_class(FootSkateCleanOperator)

    del bpy.types.Scene.footskate_clean_select_collection_name
