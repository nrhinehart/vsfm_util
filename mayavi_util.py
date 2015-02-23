import mayavi.mlab as mlab
import mayavi
import numpy
import ipdb, argh
import math_util as mathu
import vsfm_util as vsfmu
import util
from plyfile import PlyData

def plot_3d_coord_system(tds, **kwargs):
    q3d = mayavi.mlab.quiver3d([tds.origin[0]]*3,
                               [tds.origin[1]]*3,
                               [tds.origin[2]]*3,
                               tds.mat[:, 0].copy(),
                               tds.mat[:, 1].copy(),
                               tds.mat[:, 2].copy(),
                               mode = 'arrow',
                               **kwargs)
    return q3d

def ply_plot(ply_file, opacity = 1, color = (1,1,1)):
    ply = PlyData.read(ply_file)

    '''
    Plot vertices and triangles from a PlyData instance. Assumptions:
        `ply' has a 'vertex' element with 'x', 'y', and 'z'
            properties;
        `ply' has a 'face' element with an integral list property
            'vertex_indices', all of whose elements have length 3.
    '''
    vertex = ply['vertex'].data

    (x, y, z) = (vertex[t] for t in ('x', 'y', 'z'))

    # mlab.points3d(x, y, z, color=(1, 1, 1), mode='point')

    tri_idx = ply['face'].data['vertex_indices']
    idx_dtype = tri_idx[0].dtype

    triangles = numpy.fromiter(tri_idx, [('data', idx_dtype, (3,))],
                               count=len(tri_idx))['data']

    mesh = mlab.triangular_mesh(x, y, z, triangles,
                                color=color,
                                opacity = opacity)
    return mesh

def plot_normal_plane_to_3d_vector_at_loc(normal, loc, extent = 1.0, *args, **kwargs):
    mesh_points = generate_normal_plane_to_3d_vector_at_loc(normal, loc, extent = 1.0)
    
    mesh = mlab.triangular_mesh(mesh_points[:, 0], 
                                mesh_points[:, 1], 
                                mesh_points[:, 2],
                                [(0, 1, 2), (0, 1, 3), (1, 2, 3)],
                                *args,
                                **kwargs)
    return mesh

def plot_cameras(cams, **kwargs):
    cam_positions = numpy.array([_.camera_position for _ in cams])
    orientations = numpy.zeros(cam_positions.shape, dtype = numpy.float64)
    for c_idx, c in enumerate(cams):
        orientations[c_idx, :] = c.get_orientation_vec()
    cam_positions_mlab = mayavi.mlab.quiver3d(cam_positions[:, 0],
                                              cam_positions[:, 1],
                                              cam_positions[:, 2],
                                              orientations[:, 0],
                                              orientations[:, 1],
                                              orientations[:, 2],
                                              mode = 'arrow',
                                              reset_zoom = False,
                                              scale_factor = 2,
                                              **kwargs)
    return cam_positions_mlab

def load_and_plot_nvm_cams(nvm_file = None):


    models, points = vsfmu.extract_cameras_and_points_from_nvm(nvm_file)
    cams = models[0]
    
    cam_positions = numpy.array([_.camera_position for _ in cams])
    keypoints = numpy.array([_.position for _ in points])
    keypoint_colors = numpy.array([_.color_norm for _ in points])
    orientations = numpy.zeros(cam_positions.shape, dtype = numpy.float64)
    for c_idx, c in enumerate(cams):
        orientations[c_idx, :] = c.get_orientation_vec()

    cam_positions_mlab = mayavi.mlab.quiver3d(cam_positions[:, 0],
                                              cam_positions[:, 1],
                                              cam_positions[:, 2],
                                              orientations[:, 0],
                                              orientations[:, 1],
                                              orientations[:, 2],
                                              mode = 'arrow',
                                              scale_factor = .5)

    scalars = numpy.arange(0, cam_positions.shape[0])
    color_by_scalars(cam_positions_mlab, scalars)


    keypoints_mlab = mayavi.mlab.points3d(keypoints[:, 0],
                                          keypoints[:, 1],
                                          keypoints[:, 2],
                                          mode = 'point')
    color_points_by_scalars(keypoints_mlab, keypoint_colors)

    return cams, points, cam_positions_mlab, keypoints_mlab

def plot_cuboid_from_extrema(tds_floor,
                             dim1min, dim1max, dim2min, dim2max,
                             thickness = 0.05,
                             color = (0, 0, .5), 
                             color_outline = (0, 1, 0),
                             opacity = 0.3,
                             **kwargs):
    
    

    thickness_vec = -thickness * tds_floor.z

    dim1min_b = dim1min + thickness_vec
    dim1max_b = dim1max + thickness_vec
    dim2min_b = dim2min + thickness_vec
    dim2max_b = dim2max + thickness_vec

    floor_span_max = numpy.vstack((dim1min, dim1max, dim2min, dim2max,
                                   dim1min_b, dim1max_b, dim2min_b, dim2max_b))
    
    mayavi.mlab.triangular_mesh(floor_span_max[:, 0][:, numpy.newaxis], 
                                floor_span_max[:, 1][:, numpy.newaxis],
                                floor_span_max[:, 2][:, numpy.newaxis],
                                [(0, 1, 3), (0, 1, 2),  #top
                                 (0, 2, 4), (2, 4, 6),  #side 1
                                (0, 3, 4), (3, 7, 4),  #side 2
                                (1, 3, 5), (3, 5, 7),  #side 3
                                (1, 2, 5), (2, 5, 6),  #side 4
                                 (4, 5, 7), (4, 5, 6)], #bottom
                                color = color,
                                opacity = opacity,
                                representation = 'surface',
                                **kwargs)

    span_in_order = numpy.vstack((dim1min, 
                                  dim2min, 
                                  dim1max, 
                                  dim2max,
                                  dim1min,
                                  dim1min_b, #down
                                  dim2min_b, dim2min, dim2min_b, #up-down
                                  dim1max_b, dim1max, dim1max_b, #up-down
                                  dim2max_b, dim2max, dim2max_b, #up-down
                                  dim1min_b)) 

    frame = mayavi.mlab.plot3d(span_in_order[:, 0],
                               span_in_order[:, 1],
                               span_in_order[:, 2],
                               color = color_outline,
                               tube_radius = thickness / 5.0)

def color_points_by_scalars(mlab_points, sc):
    mlab_points.mlab_source.dataset.point_data.scalars = sc
    mlab.draw()

def color_by_scalars(mlab_object, sc):
    mlab_object.mlab_source.scalars = sc
    mlab_object.glyph.color_mode = 'color_by_scalar'

def color_by_vectors(mlab_object, vc):
    mlab_object.mlab_source.vectors = vc
    mlab_object.glyph.color_mode = 'color_by_vector'

if __name__ == '__main__':
    parser = argh.ArghParser()
    parser.add_commands([load_and_plot_nvm_cams])
    util.ipdbwrap(parser.dispatch)()



