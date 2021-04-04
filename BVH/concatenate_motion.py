import bpy
from . import motion_editing


class ConcatenateMotionOperator(bpy.types.Operator):
    bl_idname = "concatenate_motion.keyframe"
    bl_label = "Concatenate two motion"
    bl_description = "select two motion collection and concatenate new motion"

    @classmethod
    def poll(cls, context):
        # path_animation is not empty
        collection_name_1 = context.scene.select_concatenate_collection_name_1
        collection_name_2 = context.scene.select_concatenate_collection_name_2

        motion_path_animation_1 = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name_1)
        motion_path_animation_2 = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name_2)

        return motion_path_animation_1 != None and motion_path_animation_2 != None

    def execute(self, context):
        collection_name_1 = context.scene.select_concatenate_collection_name_1
        collection_name_2 = context.scene.select_concatenate_collection_name_2

        motion_path_animation_1 = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name_1)
        motion_path_animation_2 = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name_2)

        if motion_path_animation_1 != None and motion_path_animation_2 != None and motion_path_animation_1.bvh_parser.compare_to(
                motion_path_animation_2.bvh_parser) == False:
            return {'CANCELLED'}

        new_motion_path_animation = motion_path_animation_1.clone()
        new_motion_path_animation.bvh_parser.motion.concatenate(new_motion_path_animation.bvh_parser,
                                                                motion_path_animation_2.bvh_parser.motion)

        new_motion_path_animation.create_path()

        return {'FINISHED'}

    def concatenate(self, motion_path_animation_1, motion_path_animation_2):
        pass


def draw(context, layout):
    row = layout.row()
    # select collection will assign to context.scene.select_concatenate_collection_name_1 and context.scene.select_concatenate_collection_name_2
    # and search from bpy.data.collections
    row.prop_search(
        data=context.scene,
        property="select_concatenate_collection_name_1",
        search_data=bpy.data,
        search_property="collections",
        text="animation")

    row.prop_search(
        data=context.scene,
        property="select_concatenate_collection_name_2",
        search_data=bpy.data,
        search_property="collections",
        text="animation")

    row = layout.row()
    row.operator('concatenate_motion.keyframe', text="Concatenate motion")


def register():
    bpy.utils.register_class(ConcatenateMotionOperator)

    bpy.types.Scene.select_concatenate_collection_name_1 = bpy.props.StringProperty()
    bpy.types.Scene.select_concatenate_collection_name_2 = bpy.props.StringProperty()


def unregister():
    bpy.utils.unregister_class(ConcatenateMotionOperator)

    del bpy.types.Scene.select_concatenate_collection_name_1
    del bpy.types.Scene.select_concatenate_collection_name_2
