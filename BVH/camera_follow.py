import bpy
from . import motion_editing
from . import bvh


class CameraFollowOperator(bpy.types.Operator):
    bl_idname = "camera_follow.joint"
    bl_label = "Following a joint in animation"
    bl_description = "select one motion collection and joint and use camera to follow"

    selected_animation = None

    def load_joint(self, context):
        enum_items = []

        motion_path_animation = CameraFollowOperator.selected_animation
        if motion_path_animation != None:
            for node in motion_path_animation.bvh_parser.node_list:
                # real value, display name, discription
                enum_items.append((str(node.in_list_index), node.joint_name, node.joint_name, "", node.in_list_index))

        if len(enum_items) == 0:
            enum_items.append(("None", "None", "None"))

        return enum_items

    camera_follow_joint = bpy.props.EnumProperty(name="camera_follow_joint", items=load_joint)

    @classmethod
    def poll(cls, context):
        collection_name = context.scene.camera_follow_select_collection_name

        motion_path_animation = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name)

        return motion_path_animation != None

    def invoke(self, context, event):
        wm = context.window_manager

        collection_name = bpy.context.scene.camera_follow_select_collection_name
        motion_path_animation = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name)

        CameraFollowOperator.selected_animation = motion_path_animation

        return wm.invoke_props_dialog(self)

    def execute(self, context):

        def is_valid_foot_node(node):
            return node != None

        motion_path_animation = CameraFollowOperator.selected_animation
        if motion_path_animation != None:
            camera_follow_joint_node = motion_path_animation.bvh_parser.node_list[int(self.camera_follow_joint)]

            if is_valid_foot_node(camera_follow_joint_node) == False:
                self.report({"ERROR"}, "Illegal joint node!")
                return {"CANCELLED"}

            # TODO
            arm_ob_target = bpy.data.objects[motion_path_animation.bvh_parser.name]
            bone_subtarget = camera_follow_joint_node.in_edit_bone_name

            camera = bpy.context.scene.camera
            camera.data.clip_end = 2000

            copy_location = camera.constraints.get("COPY_LOCATION")
            if copy_location == None:
                copy_location = camera.constraints.new(type="COPY_LOCATION")
                copy_location.name = "COPY_LOCATION"

            track_to = camera.constraints.get("TRACK_TO")
            if track_to == None:
                track_to = camera.constraints.new(type="TRACK_TO")
                track_to.name = "TRACK_TO"

            limit_distance = camera.constraints.get("LIMIT_DISTANCE")
            if limit_distance == None:
                limit_distance = camera.constraints.new(type="LIMIT_DISTANCE")
                limit_distance.name = "LIMIT_DISTANCE"

            # TODO
            copy_location.target = arm_ob_target
            copy_location.subtarget = bone_subtarget
            copy_location.use_x = copy_location.use_y = copy_location.invert_z = copy_location.use_offset = copy_location.mute = False
            copy_location.use_z = True

            track_to.target = arm_ob_target
            track_to.subtarget = bone_subtarget
            track_to.track_axis = "TRACK_NEGATIVE_Z"
            track_to.up_axis = "UP_Y"
            track_to.mute = False

            limit_distance.target = arm_ob_target
            limit_distance.subtarget = bone_subtarget
            limit_distance.distance = bpy.context.scene.camera_offset_from_joint
            limit_distance.limit_mode = 'LIMITDIST_OUTSIDE'
            limit_distance.mute = False

        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.label(text="Select joint to follow")

        row = layout.row()
        row.prop(self, "camera_follow_joint", text="Joint")

    def replace_animation(self, motion_path_animation, left_foot_node, right_foot_node):
        self.solve_foot_node(motion_path_animation, left_foot_node)
        self.solve_foot_node(motion_path_animation, right_foot_node)

def draw(context, layout):
    row = layout.row()
    row.label(text="Camera follow")

    row = layout.row()
    row.prop_search(
        data=context.scene,
        property="camera_follow_select_collection_name",
        search_data=bpy.data,
        search_property="collections",
        text="animation")

    row = layout.row()
    row.operator("camera_follow.joint", text="Follow joint")

    row = layout.row()
    row.prop(bpy.context.scene, "camera_offset_from_joint", text="Camera offset from joint")


def register():
    bpy.utils.register_class(CameraFollowOperator)

    bpy.types.Scene.camera_follow_select_collection_name = bpy.props.StringProperty()
    bpy.types.Scene.camera_offset_from_joint = bpy.props.FloatProperty(default=100.0, min=0.1)


def unregister():
    bpy.utils.unregister_class(CameraFollowOperator)

    del bpy.types.Scene.camera_follow_select_collection_name
    del bpy.types.Scene.camera_offset_from_joint