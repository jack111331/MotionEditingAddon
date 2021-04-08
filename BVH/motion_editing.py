import bpy
from mathutils import Matrix, Vector

from ..bspline_regression.uniform_bspline import UniformBSpline
from ..bspline_regression.fit_uniform_bspline import UniformBSplineLeastSquaresOptimiser
import scipy.spatial
import numpy as np
import bisect

from . import bvh


def convert_to_arc_length(cubic_bspline_curve, u):
    def calculate_distance(point_1, point_2):
        velocity = np.asarray(point_1) - np.asarray(point_2)
        return np.sqrt(velocity.dot(velocity))

    total_distance = 0.0
    for i in range(1, len(cubic_bspline_curve)):
        total_distance += calculate_distance(cubic_bspline_curve[i], cubic_bspline_curve[i - 1])
    scale = u[-1]
    s = [0.0]
    for i in range(1, len(cubic_bspline_curve)):
        s.append(s[i-1] + (calculate_distance(cubic_bspline_curve[i], cubic_bspline_curve[i - 1]) / total_distance))

    new_u = [0.0]
    per_portion = 1 / len(cubic_bspline_curve)
    current_portion = per_portion
    for i in range(1, len(cubic_bspline_curve)-1):
        # Find and interpolate t
        left_idx = bisect.bisect_right(s, current_portion) - 1
        if left_idx == len(u) - 1:
            new_u.append(u[left_idx])
        else:
            right_idx = left_idx + 1
            right_portion = (current_portion - s[left_idx]) / (s[right_idx] - s[left_idx])
            new_u.append(u[left_idx] * (1.0 - right_portion) + u[right_idx] * right_portion)
        current_portion = current_portion + per_portion
    new_u.append(scale)
    return new_u


def create_poly_curve(context, collection, name, points):
    # create the Curve Datablock
    if name in bpy.data.curves.keys():
        curve_data = bpy.data.curves[name]
    else:
        curve_data = bpy.data.curves.new(name, type='CURVE')
        curve_data.dimensions = '3D'
    # curveData.resolution_u = 3

    # map coords to spline
    curve_data.splines.clear()
    polyline = curve_data.splines.new('POLY')
    polyline.points.add(len(points) - 1)

    for i, point in enumerate(points):
        x, y, z = point
        polyline.points[i].co = (x, y, z, 1)

    # create Object
    curve_ob = bpy.data.objects.new(name, curve_data)
    curve_data.bevel_depth = 0.01

    # attach to scene and validate context
    collection.objects.link(curve_ob)
    context.view_layer.objects.active = curve_ob

    return curve_ob


def create_collection(parent_collection, collection_name):
    collection = bpy.data.collections.new(name=collection_name)
    parent_collection.children.link(collection)
    return collection


def solve_cubic_bspline(point_list):
    coordinate_list = []
    w = []
    for point in point_list:
        coordinate_list.append([point.co.xyz[0], point.co.xyz[1], point.co.xyz[2]])
        w.append([1.0, 1.0, 1.0])
    if len(coordinate_list) > 100:
        control_point_amount = 10
    else:
        control_point_amount = len(coordinate_list) // 8
    if control_point_amount < 5:
        control_point_amount = 5

    c = UniformBSpline(3, control_point_amount, 3, is_closed=False)
    X = np.linspace(coordinate_list[0], coordinate_list[-1], num=control_point_amount)
    u0 = c.uniform_parameterisation(16)
    D = scipy.spatial.distance.cdist(coordinate_list, c.M(u0, X))
    u = u0[D.argmin(axis=1)]
    (u1, X1) = UniformBSplineLeastSquaresOptimiser(c, 'dn').minimise(
        coordinate_list, w, 0.1, u, X)
    return X1, u1


def create_cube(collection, name, position, scale=1.0):
    me = bpy.data.meshes.new(name)
    verts = [
        (-0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (0.5, 0.5, -0.5), (0.5, -0.5, -0.5),
        (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5)]
    faces = [
        (0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1),
        (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]

    scale_verts = []
    for v in verts:
        scale_verts.append((scale * v[0], scale * v[1], scale * v[2]))

    me.from_pydata(scale_verts, [], faces)
    me.update(calc_edges=True)

    ob = bpy.data.objects.new(name, me)
    ob.location = position
    collection.objects.link(ob)

    return ob


def interpolate_cubic_bspline_curve(control_points, u):
    c = UniformBSpline(3, len(control_points), 3, is_closed=False)
    return c.M(u, control_points)


def create_cubic_bspline_curve(context, collection, control_points, name, u):
    cubic_bspline_curve = interpolate_cubic_bspline_curve(control_points, u)
    # arc-length the curve and regenerate it
    new_u = convert_to_arc_length(cubic_bspline_curve, u)
    cubic_bspline_curve = interpolate_cubic_bspline_curve(control_points, new_u)
    return create_poly_curve(context, collection, name, cubic_bspline_curve)


def compute_line_orientation(front, world_up=Vector((0, 1, 0))):
    z = front
    x = world_up.cross(z)
    y = z.cross(x)

    x.normalize()
    y.normalize()
    z.normalize()

    rotation_matrix = Matrix((x, y, z)).transposed().to_4x4()
    return rotation_matrix


class MotionPathAnimation:
    path_animations = []

    def __init__(self, bvh_parser, context):
        self.bvh_parser = bvh_parser
        self.context = context
        self.collection = create_collection(context.scene.collection, bvh_parser.name)
        self.path_collection = None
        self.control_point_collection = None
        self.initial_motion_curve = None
        self.initial_path = None
        self.new_path = None
        self.new_motion_curve = None
        self.new_motion_data = None
        self.initial_to_new_path_transform_matrix_list = None
        self.is_updating_curve = False
        self.u = None

    def clone(self):
        new_bvh_parser = self.bvh_parser.clone()
        new_bvh_parser.name = new_bvh_parser.name + "$copy"
        motion_path_animation = MotionPathAnimation(new_bvh_parser, self.context)
        bvh.bvh_node_dict2armature(self.context, new_bvh_parser.name, new_bvh_parser)
        motion_path_animation.u = self.u[:]
        MotionPathAnimation.path_animations.append(motion_path_animation)
        return motion_path_animation

    @classmethod
    def get_path_from_collection_name(cls, collection_name):
        for i in range(len(cls.path_animations)):
            if cls.path_animations[i].bvh_parser.name == collection_name:
                return cls.path_animations[i]
        return None

    @classmethod
    def add_path_animation_from_parser(cls, context, parser):
        motion_path_animation = MotionPathAnimation(parser, context)
        MotionPathAnimation.path_animations.append(motion_path_animation)
        return motion_path_animation

    def create_path(self):
        self.path_collection = create_collection(self.collection, self.bvh_parser.name + ".path")

        self.initial_motion_curve = self.create_initial_motion_curve()
        self.initial_path, self.new_path = self.create_path_curve()
        self.new_motion_curve = self.create_new_motion_curve()

        # register handler to trigger control point movement event
        from bpy.app.handlers import persistent

        @persistent
        def change_cotrol_point_handler(scene):
            for ob in self.context.selected_objects:
                # is control point
                # if ob.name in {point.name for point in self.path_c_points_ob}:
                if ob.users_collection[0] is self.control_point_collection:
                    # update bspline
                    self.update_new_path_and_motion_curve()
                    break

        # clear handler, if only one animation you can enable this!
        # bpy.app.handlers.depsgraph_update_pre.clear()

        bpy.app.handlers.depsgraph_update_pre.append(change_cotrol_point_handler)

        return {'FINISHED'}

    #
    def create_initial_motion_curve(self):
        curve = []

        # use root to track curve
        root_pos_and_orientation_list = self.bvh_parser.motion.generate_root_pos_and_orientation(
            self.bvh_parser.node_list)

        for i in range(len(root_pos_and_orientation_list)):
            curve.append(root_pos_and_orientation_list[i][:3])

        return create_poly_curve(self.context, self.path_collection, "initial_motion_curve", curve)

    def create_path_curve(self):

        self.control_point_collection = create_collection(self.path_collection,
                                                          self.bvh_parser.name + ".control_points")

        c_points, self.u = solve_cubic_bspline(self.initial_motion_curve.data.splines[0].points.values())
        for i in range(len(c_points)):
            create_cube(self.control_point_collection, "c_" + str(i), c_points[i], 10.0)

        return (
            create_cubic_bspline_curve(self.context, self.path_collection, c_points, "init_path", self.u),
            create_cubic_bspline_curve(self.context, self.path_collection, c_points, "new_path", self.u))

    #
    def create_new_motion_curve(self):
        self.initial_to_new_path_transform_matrix_list = []
        init_curve = self.initial_path.data.splines[0].points.values()
        new_curve = self.new_path.data.splines[0].points.values()

        # to_bvh_coordinate_system_matrix = self.bvh_parser.get_blender_world_to_bvh_world_transform_matrix(0)

        for i in range(self.bvh_parser.motion.frame_amount):
            p0 = init_curve[i].co.xyz
            p = new_curve[i].co.xyz

            init_translation_matrix = Matrix.Translation(p0)
            new_translation_matrix = Matrix.Translation(p)

            initial_rotation_matrix = Matrix.Identity(4)
            new_rotation_matrix = Matrix.Identity(4)
            if i == 0:
                f0 = init_curve[i + 1].co.xyz - init_curve[i].co.xyz
                f = new_curve[i + 1].co.xyz - new_curve[i].co.xyz
            else:
                f0 = init_curve[i].co.xyz - init_curve[i - 1].co.xyz
                f = new_curve[i].co.xyz - new_curve[i - 1].co.xyz

            if f0.length > 0.001:
                initial_rotation_matrix = compute_line_orientation(f0)
            if f.length > 0.001:
                new_rotation_matrix = compute_line_orientation(f)

            # Transform to new
            transform_matrix = new_translation_matrix @ new_rotation_matrix \
                               @ initial_rotation_matrix.inverted() @ init_translation_matrix.inverted()

            self.initial_to_new_path_transform_matrix_list.append(transform_matrix)

        self.new_motion_data = self.bvh_parser.motion.generate_all_transformed_frame(
            self.bvh_parser, root_transform_matrix_list=self.initial_to_new_path_transform_matrix_list)
        curve = []
        aggregate_frame = bvh.AggregateFrame()
        aggregate_frame.assign_node_list(self.bvh_parser.node_list)
        root_pos = self.bvh_parser.motion.generate_root_pos_and_orientation(self.bvh_parser.node_list, root_transform_matrix_list=self.initial_to_new_path_transform_matrix_list)
        for frame_i in range(len(self.new_motion_data.frame_list)):
            aggregate_frame.assign_frame(self.new_motion_data.frame_list[frame_i])
            curve.append(aggregate_frame.get_loc(0))
            # curve.append(root_pos[frame_i][:3])

        return create_poly_curve(self.context, self.path_collection, "new_motion_curve", curve)

    def update_new_path_and_motion_curve(self):
        if self.is_updating_curve == False:
            self.is_updating_curve = True
            bpy.data.objects.remove(self.new_path)
            bpy.data.objects.remove(self.new_motion_curve)

            c_points = []
            for c_point_ob in self.control_point_collection.all_objects.values():
                c_points.append(c_point_ob.location.xyz)

            self.new_path = create_cubic_bspline_curve(self.context, self.path_collection
                                                       , c_points, "new_path", self.u)
            self.new_motion_curve = self.create_new_motion_curve()
            self.is_updating_curve = False
