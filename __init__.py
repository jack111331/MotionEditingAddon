import bpy
from bpy_extras.io_utils import ImportHelper, orientation_helper, axis_conversion
from .BVH import bvh

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
        return bvh.load(context, **keywords)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(BVHImport.bl_idname, text="Motion Path Editing(.bvh)")

def register():
    bpy.utils.register_class(BVHImport)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

def unregister():
    bpy.utils.unregister_class(BVHImport)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

if __name__ == "__main__":
    register()