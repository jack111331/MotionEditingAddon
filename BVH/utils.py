__all__ = (
    "armature_helper"
)

import bpy
from bpy.props import (
    EnumProperty,
    StringProperty,
)

def armature_helper(armature_string=''):
    """
    A decorator for import/export classes, generating properties needed by attach to armature object.
    """
    def wrapper(cls):
        # Without that, we may end up adding those fields to some **parent** class' __annotations__ property
        # (like the ImportHelper or ExportHelper ones)! See T58772.
        if "__annotations__" not in cls.__dict__:
            setattr(cls, "__annotations__", {})

        def load_armature_ob(self, context):
            enum_items = [("None", "None", "None")]
            arm_ob_cnt = 1
            for arm_ob in bpy.data.objects:
                # real value, display name, discription
                if arm_ob.type == "ARMATURE":
                    enum_items.append(
                        (arm_ob.name, arm_ob.name, arm_ob.name, "", arm_ob_cnt))
                    arm_ob_cnt = arm_ob_cnt + 1

            return enum_items

        def _update_armature(self, _context):
            print("Update", self.armature_string)

        cls.__annotations__['armature_string'] = EnumProperty(
            name="Armature object",
            items=load_armature_ob,
            update=_update_armature,
        )

        return cls

    return wrapper