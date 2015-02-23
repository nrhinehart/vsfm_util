import numpy
import type_util, os
import mayavi
import mayavi.mlab
import tvtk.api

class bundler_cam(object):
    @type_util.member_initializer
    def __init__(self, f, k1, k2, R, t):
        self.valid = True
        if self.f == 0:
            self.valid = False
        self.view_vec =  - R[2, :]
        self.world_pos = - numpy.dot(R.T, t)

class bundler_keypoint(object):
    @type_util.member_initializer
    def __init__(self, position, color, view_list):
        self.in_n_cams = view_list[0]
        self.xys = []

        for cam_idx in range(int(self.in_n_cams)):
            self.xys.append(view_list[4 * cam_idx + 3 : 
                                      4 * cam_idx + 5])


class bundler_sfm_file(object):
    @type_util.member_initializer
    def __init__(self, fn):
        if not os.path.isfile(fn):
            raise ArgumentError("file doesn't exist: {}".format(fn))
            
        lines = [x.strip().split(' ') for x in open(fn, 'r').readlines()]
        self.n_cams, self.n_points = map(int, lines[1])

        line_idx = 2

        self.valid = (self.n_cams > 0 and self.n_points > 0)
        if not self.valid:
            s = "Bundler file not valid!!! {} cameras, {} points".format(self.n_cams,
                                                                         self.n_points)
            raise RuntimeError(s)


        self.cams = []
        self.keypoints = []



        R = numpy.zeros((3, 3), dtype = numpy.float64)
        for cam_idx in range(self.n_cams):
            f, k1, k2 = map(float, lines[line_idx])
            R[0, :] = map(float, lines[line_idx + 1])
            R[1, :] = map(float, lines[line_idx + 2])
            R[2, :] = map(float, lines[line_idx + 3])
            t = numpy.asarray(map(float, lines[line_idx + 4]))

            line_idx += 5
            self.cams.append(bundler_cam(f, k1, k2, R.copy(), t))

        assert(self.n_cams == len(self.cams))
        self.cam_world_positions = numpy.vstack([x.world_pos for x in self.cams])
        self.cam_world_orientations = numpy.vstack([x.view_vec for x in self.cams])
        self.valid_cam_mask = numpy.asarray([c.valid for c in self.cams])

        assert(self.cam_world_positions.shape[1] == 3)
        assert(self.cam_world_orientations.shape[1] == 3)
        
        for point_idx in range(self.n_points):
            pos = numpy.asarray(map(float, lines[line_idx]))
            color = numpy.asarray(map(float, lines[line_idx + 1]))
            view_list = numpy.asarray(map(float, lines[line_idx + 2]))
            line_idx += 3
            self.keypoints.append(bundler_keypoint(pos, color, view_list))

        assert(self.n_points == len(self.keypoints))

        self.keypoint_world_positions = numpy.vstack([k.position for k in self.keypoints])
        self.keypoint_colors = numpy.vstack([k.color for k in self.keypoints]).astype('uint8')

    def plot(self, fig):
        self.valid_cam_points = self.cam_world_positions[self.valid_cam_mask, :]
        self.valid_cam_orientations = self.cam_world_orientations[self.valid_cam_mask, :]

        
        arrows = mayavi.mlab.quiver3d(self.valid_cam_points[:, 0],
                                      self.valid_cam_points[:, 1],
                                      self.valid_cam_points[:, 2],
                                      self.valid_cam_orientations[:, 0],
                                      self.valid_cam_orientations[:, 1],
                                      self.valid_cam_orientations[:, 2],
                                      mode = 'arrow',
                                      scale_factor = 1,
                                      figure = fig)

        points = mayavi.mlab.points3d(self.keypoint_world_positions[:, 0],
                                      self.keypoint_world_positions[:, 1],
                                      self.keypoint_world_positions[:, 2],
                                      # self.keypoint_colors[:, 0],
                                      # self.keypoint_colors[:, 1],
                                      # self.keypoint_colors[:, 2],
                                      mode = 'point')

        colors_char = tvtk.api.tvtk.UnsignedCharArray()
        colors_char.from_array(self.keypoint_colors)
        points.mlab_source.dataset.point_data.scalars = colors_char
        # points.actor.mapper.input = points.mlab_source.dataset
        points.mlab_source.dataset.modified()
        mayavi.mlab.show()
        return arrows
