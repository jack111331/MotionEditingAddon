import bpy
from bpy_extras.io_utils import ImportHelper, orientation_helper, axis_conversion
from bpy.props import BoolProperty
from .BVH import concatenate_motion
from .BVH import footskate_clean
import subprocess
import os

# Install dependency in blender environment
py_exec = bpy.app.binary_path_python
if not os.path.exists(os.path.join(str(py_exec)[:-14] + 'lib', 'scipy')) or not os.path.exists(os.path.join(str(py_exec)[:-14] + 'lib', 'sympy')):
    print('Installing dependency')
    # ensure pip is installed
    subprocess.call([str(py_exec), "-m", "ensurepip", "--user"])
    # update pip
    subprocess.call([str(py_exec), "-m", "pip", "install", "--upgrade", "pip"])
    # install packages
    subprocess.call([str(py_exec), "-m", "pip", "install", f"--target={str(py_exec)[:-14]}" + "lib", "scipy"])
    subprocess.call([str(py_exec), "-m", "pip", "install", f"--target={str(py_exec)[:-14]}" + "lib", "sympy"])

from .BVH import bvh, motion_editing

bl_info = {
    "name": "BVH parser and motion path editing",
    "author": "Edge",
    "version": (1, 0, 0),
    "blender": (2, 81, 6),
    "location": "File > Import",
    "description": "import .bvh file",
    "warning": "",
    "wiki_url": "",
    "support": 'TESTING',
    "category": "Import-Export",
}


@orientation_helper(axis_forward='-Z', axis_up='Y')
class BVHImport(bpy.types.Operator, ImportHelper):
    bl_idname = "edge_bvh_workspace.bvh"
    bl_label = "Import BVH data"
    bl_options = {'REGISTER', 'UNDO'}

    # ImportHelper mixin class uses this
    filename_ext = ".bvh"

    filter_glob: bpy.props.StringProperty(
        default="*.bvh",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    def execute(self, context):
        print("Opening: ", self.filepath)
        keywords = self.as_keywords(
            ignore=(
                "axis_forward",
                "axis_up",
                "filter_glob",
            )
        )
        global_matrix = axis_conversion(
            from_forward=self.axis_forward,
            from_up=self.axis_up,
        ).to_4x4()

        keywords["global_matrix"] = global_matrix
        parser = bvh.load(context, **keywords)
        motion_path_animation = motion_editing.MotionPathAnimation.add_path_animation_from_parser(context, parser,
                                                                                                  global_matrix)
        return motion_path_animation.create_path()


class ApplyAnimation(bpy.types.Operator):
    bl_idname = "generated_animation.keyframe"
    bl_label = "apply generated keyframe animation"
    bl_description = "select animation collection and apply motion on it"

    @classmethod
    def poll(cls, context):
        # path_animation is not empty
        collection_name = context.scene.select_collection_name

        motion_path_animation = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name)

        return motion_path_animation != None

    def execute(self, context):
        collection_name = context.scene.select_collection_name

        motion_path_animation = motion_editing.MotionPathAnimation.get_path_from_collection_name(collection_name)
        arm_ob = bpy.data.objects[motion_path_animation.bvh_parser.name]
        if motion_path_animation != None:
            if context.scene.use_origin_motion == True:
                motion_path_animation.bvh_parser.motion.apply_motion_on_armature(context, motion_path_animation.bvh_parser.node_list, arm_ob, 1, motion_path_animation.global_matrix)
            else:
                motion_path_animation.new_motion_data.apply_motion_on_armature(context, motion_path_animation.bvh_parser.node_list, arm_ob, 1, motion_path_animation.global_matrix)

        return {'FINISHED'}

class PathAnimationGeneratePanel(bpy.types.Panel):
    bl_idname = "PATH_PT_ANIMATION_GENERATE"
    bl_label = "path animation panel"
    bl_category = "Motion Animation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        # scene = context.scene

        row = layout.row()
        # select collection will assign to context.scene.select_animation
        # and search from bpy.data.collections
        row.prop_search(
            data=context.scene,
            property="select_collection_name",
            search_data=bpy.data,
            search_property="collections",
            text="animation")

        row = layout.row()
        row.prop(context.scene, "use_origin_motion", text="Use origin motion")

        row = layout.row()
        row.operator('generated_animation.keyframe', text="apply animation")

        # draw concatenate motion panel
        concatenate_motion.draw(context, layout)
        footskate_clean.draw(context, layout)



# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(BVHImport.bl_idname, text="Motion Path Editing(.bvh)")


def register():
    bpy.utils.register_class(BVHImport)
    bpy.utils.register_class(ApplyAnimation)
    bpy.utils.register_class(PathAnimationGeneratePanel)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

    # Register this property so that we can easily access the selected collection through panel
    bpy.types.Scene.select_collection_name = bpy.props.StringProperty()
    bpy.types.Scene.use_origin_motion = BoolProperty(name="use_origin_motion")

    concatenate_motion.register()
    footskate_clean.register()



def unregister():
    bpy.utils.unregister_class(BVHImport)
    bpy.utils.unregister_class(ApplyAnimation)
    bpy.utils.unregister_class(PathAnimationGeneratePanel)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

    concatenate_motion.unregister()
    footskate_clean.unregister()


if __name__ == "__main__":
    register()
