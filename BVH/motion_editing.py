import bpy
from mathutils import Matrix

def create_poly_curve(context, collection, name, points, global_matrix):
    # create the Curve Datablock
    curve_data = bpy.data.curves.new(name, type='CURVE')
    curve_data.dimensions = '3D'
    # curveData.resolution_u = 3

    # map coords to spline
    polyline = curve_data.splines.new('POLY')
    polyline.points.add(len(points) - 1)

    for i, point in enumerate(points):
        x, y, z = point
        polyline.points[i].co = (x, y, z, 1)

    # create Object
    curve_ob = bpy.data.objects.new(name, curve_data)
    curve_data.bevel_depth = 0.01

    curve_ob.matrix_world = global_matrix
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

    # attach to scene and validate context
    collection.objects.link(curve_ob)
    context.view_layer.objects.active = curve_ob

    return curve_ob


def create_collection(parent_collection, collection_name):
    collection = bpy.data.collections.new(name=collection_name)
    parent_collection.children.link(collection)
    return collection


class MotionPathAnimation:
    path_animations = []

    def __init__(self, bvh_parser, context, global_matrix=None):
        self.bvh_parser = bvh_parser
        self.context = context
        self.collection = create_collection(context.scene.collection, bvh_parser.name)
        self.path_collection = None
        self.initial_motion = None
        self.initial_path = None
        self.new_path = None
        self.new_motion = None
        if global_matrix == None:
            self.global_matrix = Matrix.Identity(4)
        else:
            self.global_matrix = global_matrix

    @classmethod
    def add_path_animation_from_parser(cls, context, parser, global_matrix):
        motion_path_animation = MotionPathAnimation(parser, context, global_matrix)
        MotionPathAnimation.path_animations.append(motion_path_animation)
        return motion_path_animation

    def create_path(self):
        self.path_collection = create_collection(self.collection, self.bvh_parser.name + ".path")

        self.initial_motion = self.create_initial_motion_curve()
        # self.initial_path, self.new_path = self.create_path_curve()
        # self.new_motion = self.createNewMotionCurve()


    #
    def create_initial_motion_curve(self):
        curve = []

        # use root to track curve
        root_pos_and_orientation_list = self.bvh_parser.motion_list[0].generate_root_pos_and_orientation(self.bvh_parser.node_list)

        for i in range(len(root_pos_and_orientation_list)):
            curve.append(root_pos_and_orientation_list[i][:3])

        return create_poly_curve(self.context, self.path_collection, "initial_motion", curve, self.global_matrix)

    # def create_path_curve(self):
    #
    #     self.control_points = create_collection(self.path_collection, self.bvh_parser.name + ".control_points")
    #
    #     c_points, self.t = solveCubicBspline(self.initial_motion.data.splines[0].points.values())
    #     for i in range(len(c_points)):
    #         createCube(self.control_points, "c_" + str(i), c_points[i], 10.0)
    #
    #     return (
    #         createCubicBspline(self.context, self.path, c_points, "init_path", self.t),
    #         createCubicBspline(self.context, self.path, c_points, "new_path", self.t))

    # #
    # def createNewMotionCurve(self):
    #     # use root to track curve
    #     root = NodeBVH.getRoot(self.nodes_bvh)
    #
    #     curve = []
    #
    #     self.init_to_new_matrixs = []
    #     init_curve = self.init_path.data.splines[0].points.values()
    #     new_curve = self.new_path.data.splines[0].points.values()
    #     for i in range(self.frames_bvh):
    #         p0 = init_curve[i].co
    #         p = new_curve[i].co
    #
    #         P0 = Matrix.Translation(p0)
    #         P = Matrix.Translation(p)
    #
    #         R0 = Matrix.Identity(4)
    #         R = Matrix.Identity(4)
    #         if (i == 0):
    #             f0 = init_curve[i + 1].co - init_curve[i].co
    #             f = new_curve[i + 1].co - new_curve[i].co
    #         else:
    #             f0 = init_curve[i].co - init_curve[i - 1].co
    #             f = new_curve[i].co - new_curve[i - 1].co
    #
    #         if f0.length > 0.001:
    #             R0 = computeOrientation(f0, Vector([0, 0, 1]))
    #         if f.length > 0.001:
    #             R = computeOrientation(f, Vector([0, 0, 1]))
    #
    #         matrix = P @ R @ R0.inverted() @ P0.inverted()
    #
    #         self.init_to_new_matrixs.append(matrix)
    #
    #         NodeBVH.updateNodesWorldPosition(self.nodes_bvh, i, matrix)
    #         curve.append((root.world_head))
    #
    #     return createPolyCurve(self.context, self.path, "new_motion", curve)
    #
    # #
    # def createNewReparameterPathCurve(self, path_name):
    #     c_points = []
    #     for c_point_ob in self.control_points.all_objects.values():
    #         c_points.append(c_point_ob.location.xyz)
    #
    #     Q = []
    #     for point in self.new_motion.data.splines[0].points.values():
    #         Q.append(point.co.xyz)
    #     self.re_t = computeChordLengthParameter(Q)
    #
    #     return createCubicBspline(self.context, self.path, c_points, path_name, self.re_t)
    #
    # #
    # def updateNewPathAndMotionCurve(self):
    #
    #     path_name = self.new_path.name
    #     motion_name = self.new_motion.name
    #
    #     # selected = []
    #     # for ob in self.context.selected_objects:
    #     #     if not ob.name in {path_name, motion_name}:
    #     #         selected.append(ob)
    #
    #     # # delete "path" and create new one
    #     # for ob in self.context.scene.objects:
    #     #     if ob.name in {path_name, motion_name}:
    #     #         ob.select_set(True)
    #     #     else:
    #     #         ob.select_set(False)
    #
    #     # bpy.ops.object.delete()
    #
    #     # for ob in selected:
    #     #     ob.select_set(True)
    #
    #     bpy.data.objects.remove(self.new_path)
    #     bpy.data.objects.remove(self.new_motion)
    #
    #     c_points = []
    #     for c_point_ob in self.control_points.all_objects.values():
    #         c_points.append(c_point_ob.location.xyz)
    #
    #     self.new_path = createCubicBspline(self.context, self.path, c_points, path_name, self.t)
    #     self.new_motion = self.createNewMotionCurve()
    #
    #     # reparameter
    #     bpy.data.objects.remove(self.new_path)
    #     self.new_path = self.createNewReparameterPathCurve(path_name)
    #     bpy.data.objects.remove(self.new_motion)
    #     self.new_motion = self.createNewMotionCurve()