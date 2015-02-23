import math_util as mathu
import mayavi_util as mayaviu
import numpy, ipdb
import mayavi.mlab

class scene_object(object):
    def __init__(self, mesh, floor_coord_sys, sf = 1, xy_angular_offset = 0):
        self.mesh = mesh
        self.mls = mesh.mlab_source
        self.floor_coord_sys = floor_coord_sys
        self.fcs = floor_coord_sys

        self.surface_top_midpoint = None

        if sf is not 1:
            scale_mesh(self.mesh, sf)

        put_object_on_floor(self.mesh, self.floor_coord_sys)

        # orientation state is tracked independently of mesh
        self.orientation = numpy.array([1, 0, 0], dtype = numpy.float64)
        self.orientation_world = self.fcs.convert_points_to_world_frame(self.orientation)

        if xy_angular_offset != 0:
            self.rotate_xy(xy_angular_offset)

    def rotate_xy(self, angle):        
        R_local = rotate_mesh_in_coord_system(self.mesh, self.fcs, [0, 0, 1], angle)
        self.orientation = numpy.dot(R_local, self.orientation)
        self.orientation_world = self.fcs.convert_points_to_world_frame(self.orientation)

    # top surface points are within some delta along gravity normal, midpoint
    # is the average of them
    def calc_surface_top_midpoint(self):
        mls_points = numpy.vstack((self.mls.x, self.mls.y, self.mls.z)).T
        dists = numpy.dot(mls_points, self.floor_coord_sys.a3)
        top_point_ind = numpy.argmax(dists)

        point_on_top = mls_points[top_point_ind, :]

        mls_points_centered = mls_points - point_on_top
        dists_from_top = numpy.dot(mls_points_centered, self.floor_coord_sys.a3)
        top_points = mls_points[abs(dists_from_top) < 1e-7, :]
        self.surface_top_midpoint = top_points.mean(axis = 0)
        return self.surface_top_midpoint

    # bottom surface points are within some delta along gravity normal, midpoint
    # is the average of them
    def calc_surface_bottom_midpoint(self):
        mls_points = numpy.vstack((self.mls.x, self.mls.y, self.mls.z)).T
        dists = numpy.dot(mls_points, -self.floor_coord_sys.a3)
        top_point_ind = numpy.argmax(dists)

        point_on_top = mls_points[top_point_ind, :]

        mls_points_centered = mls_points - point_on_top
        dists_from_top = numpy.dot(mls_points_centered, self.floor_coord_sys.a3)
        bottom_points = mls_points[abs(dists_from_top) < 1e-7, :]
        self.surface_bottom_midpoint = bottom_points.mean(axis = 0)
        return self.surface_bottom_midpoint

    # put another object on top of self
    def stack_object(self, other_obj):
        if self.surface_top_midpoint is None:
            self.calc_surface_top_midpoint()
            
        other_obj_bottom = other_obj.calc_surface_bottom_midpoint()
        offset_vec = other_obj_bottom - self.surface_top_midpoint
        shift_mesh(other_obj.mesh, offset_vec)

    # scale mesh
    def scale(self, sf):
        scale_mesh(self.mesh, sf)

    # slide along floor (no Z component)
    def slide_along_floor(self, xcomp, ycomp):
        shift_mesh_in_coord_system(self.mesh, self.floor_coord_sys, numpy.array([xcomp, ycomp, 0]))

    # 
    def get_xy_orientation(self):
        #xy_o = self.fcs.project_points_to_xy_plane(self.orientation_world)
        xy_o = numpy.array([self.orientation[0], self.orientation[1], 0])
        return xy_o

    # path should contain camera pose vectors (not just localization)...
    def animate_along_path(self, path_world, save_ims = False):
        vecs = mathu.get_path_vectors(path_world)

        vecs_xy = self.fcs.zero_component_along_dimension(vecs, 2)
        xy_o = self.fcs.convert_points_to_world_frame(self.get_xy_orientation())
        
        path_world_proj = self.fcs.project_world_points_to_xy_plane(path_world)
        path_floor_proj = self.fcs.convert_points_from_world_frame(path_world_proj, apply_offset = True)

        # mayavi.mlab.points3d(path_world_proj[:, 0], 
        #                      path_world_proj[:, 1],
        #                      path_world_proj[:, 2],
        #                      scale_factor = .05)

        last_cpof = None
        path_cnt = 0
        last_vec = xy_o
        for cpof in path_floor_proj:
            if last_cpof is not None:
                offset = cpof - last_cpof
            else:
                offset = cpof

            vec = vecs_xy[path_cnt, :]
            angle_offset = mathu.angle_between_vectors(last_vec, vec)
            last_vec = vec

            rotate_mesh_in_coord_system(self.mesh, self.fcs, [0, 0, 1], angle_offset)
            slide_mesh_along_floor(self.mesh, self.fcs, offset[0], offset[1])

            if save_ims:
                mayavi.mlab.savefig("path_{:08d}.png".format(path_cnt))

            last_cpof = cpof
            path_cnt += 1

# @mayavi.mlab.animate(delay = 50)
def trace_path(fcs, path_coords, mesh, save_ims = True):
    coords_proj = mathu.project_points_to_plane(fcs.origin,
                                                fcs.a3,
                                                path_coords)
    coords_proj_offset_world = numpy.subtract(coords_proj, fcs.origin)
    coords_proj_offset_floor = fcs.convert_points_from_world_frame(coords_proj_offset_world)

    last_cpof = None
    path_cnt = 0
    for cpof in coords_proj_offset_floor:
        if last_cpof is not None:
            offset = cpof - last_cpof
        else:
            offset = cpof
        slide_mesh_along_floor(mesh, fcs, offset[0], offset[1])
        last_cpof = cpof
        if save_ims:
            mayavi.mlab.savefig("path_{:08d}.png".format(path_cnt))
        path_cnt += 1
        yield

def calc_mesh_center(mesh):
    mls = mesh.mlab_source
    xc = mls.x.mean()
    yc = mls.y.mean()
    zc = mls.z.mean()
    vc = numpy.array([xc, yc, zc])
    return vc

def scale_mesh(mesh, scale_factor = .5):
    mls = mesh.mlab_source
    mesh.mlab_source.set(x = mls.x * scale_factor,
                         y = mls.y * scale_factor,
                         z = mls.z * scale_factor)

def rotate_mesh(mesh, r = numpy.eye(3)):
    mls = mesh.mlab_source
    data = numpy.vstack((mls.x, mls.y, mls.z))

    data_r = numpy.dot(r, data)

    mesh.mlab_source.set(x = data_r[0, :],
                         y = data_r[1, :],
                         z = data_r[2, :])

def shift_mesh(mesh, offset):
    mls = mesh.mlab_source
    
    data = numpy.vstack((mls.x, mls.y, mls.z))
    data_r = (data.T - offset).T

    mesh.mlab_source.set(x = data_r[0, :],
                         y = data_r[1, :],
                         z = data_r[2, :])

def put_object_on_floor(mesh, tds_floor):
    rotate_mesh(mesh, tds_floor.mat.T)
    shift_mesh(mesh, -tds_floor.origin)
        
def rotate_mesh_in_coord_system(mesh, coord_sys, axis, angle, center = True):
    if center:
        mesh.scene.disable_render = True
        mesh_center = calc_mesh_center(mesh)
        shift_mesh(mesh, mesh_center)

    rot_mat = mathu.rotation_matrix(axis, angle)
    rotate_mesh(mesh, coord_sys.rotation_matrix_to_world_frame(rot_mat))

    if center:
        shift_mesh(mesh, -mesh_center)        
    mesh.scene.disable_render = False
    return rot_mat

def shift_mesh_in_coord_system(mesh, coord_sys, offset):
    t_world = coord_sys.convert_points_to_world_frame(-offset)
    shift_mesh(mesh, t_world)

def slide_mesh_along_floor(mesh, tds_floor, xcomp, ycomp):
    shift_mesh_in_coord_system(mesh, tds_floor, numpy.array([xcomp, ycomp, 0]))

def setup_scene_9(tds_floor, chair_mesh, table_mesh, couch_mesh):
    rotate_mesh_in_coord_system(couch_mesh, tds_floor, [0, 0, 1], numpy.pi/5)
    rotate_mesh_in_coord_system(chair_mesh, tds_floor, [0, 0, 1], 5 * numpy.pi / 4)
    rotate_mesh_in_coord_system(table_mesh, tds_floor, [0, 0, 1], - 3 * numpy.pi/8)

    slide_mesh_along_floor(couch_mesh, tds_floor, -1, .9)
    slide_mesh_along_floor(table_mesh, tds_floor, -.3, .15)

def setup_scene_8(tds_floor, chair_mesh, table_mesh, couch_mesh):
    rotate_mesh_in_coord_system(couch_mesh, tds_floor, [0, 0, 1], -3*numpy.pi/8)
    rotate_mesh_in_coord_system(chair_mesh, tds_floor, [0, 0, 1], 5 * numpy.pi / 8)
    rotate_mesh_in_coord_system(table_mesh, tds_floor, [0, 0, 1], numpy.pi/8)

    slide_mesh_along_floor(couch_mesh, tds_floor, .1, .4)
    slide_mesh_along_floor(table_mesh, tds_floor, -.4, .7)


def calc_scene_floor_from_gravity_and_points(normal, data, lowest_n = 500):
    dists = numpy.dot(data[:, :3], -normal)


    lowest_point_inds = numpy.argsort(dists)[::-1]
    
    median_lowest_point = data[lowest_point_inds[250], :3]
    # mayavi.mlab.points3d(*median_lowest_point, scale_factor = .1)


    print "projecting points to plane"
    proj_points = mathu.project_points_to_plane(median_lowest_point,
                                                normal,
                                                data[:, :3])

    #calcuate floor extent and coordinate system
    floor_center = proj_points.mean(axis = 0)
    floor_height = numpy.dot(floor_center, normal / numpy.linalg.norm(normal))

    floor_vecs = proj_points - floor_center
    floor_dists = numpy.linalg.norm(floor_vecs, axis = 1)
    furthest_floor_vec = floor_vecs[numpy.argmax(floor_dists), :]

    bvec1 = furthest_floor_vec
    bvec2 = numpy.cross(normal, furthest_floor_vec)

    v1dists = numpy.dot(floor_vecs, bvec1)
    v1as = numpy.argsort(v1dists)
    
    v2dists = numpy.dot(floor_vecs, bvec2)
    v2as = numpy.argsort(v2dists)

    v1min, v1max = floor_vecs[v1as[0], :] + floor_center, floor_vecs[v1as[-1], :] + floor_center
    v2min, v2max = floor_vecs[v2as[0], :] + floor_center, floor_vecs[v2as[-1], :] + floor_center

    ff2 = floor_vecs[v2as[-1], :]

    # set up coordinate system
    tds_floor = mathu.three_d_coord_system(floor_center, bvec1, bvec2, normal)
    # return tds_floor, v1min, v1max, v2min, v2max
    return tds_floor
    
