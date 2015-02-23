import type_util
import numpy, math
np = numpy
import ipdb, pdb



class three_d_coord_system(object):

    @type_util.member_initializer
    def __init__(self, origin, a1, a2, a3, tol = 1e-6, check_ortho = True):
        """
        Origin is in world coordinates. A{1,2,3} are orthogonal axes
        """
        self.origin = numpy.asarray(origin)
        self.a1 = a1 / numpy.linalg.norm(a1)
        self.a2 = a2 / numpy.linalg.norm(a2)
        self.a3 = a3 / numpy.linalg.norm(a3)
        self.x = self.a1
        self.y = self.a2
        self.z = self.a3

        if check_ortho:
            if abs(numpy.dot(a1, a2)) > tol or \
               abs(numpy.dot(a1, a3)) > tol or \
               abs(numpy.dot(a2, a3)) > tol:
                raise RuntimeError("Axes are not orthogonal")
        
        self.mat = numpy.vstack((self.a1, self.a2, self.a3))
        
    # assumes points are in world reference frame (NOT SELF)
    # returns points on xy_plane in world coordinates
    def project_world_points_to_xy_plane(self, points):
        if points.ndim == 1:
            points = points[numpy.newaxis, :]
        vectors = points - self.origin
        dists = numpy.dot(vectors, self.z)
        n_zs = numpy.tile(self.z[:, numpy.newaxis], dists.shape[0])
        points_proj = points - (dists * n_zs).T
        return points_proj

    # point are 3-dimensional world frame points, result is Nx3
    def convert_points_from_world_frame(self, points, apply_offset = False):
        if points.ndim == 1:
            points = points[numpy.newaxis, :]
        if 3 not in points.shape or points.ndim != 2:
            raise RuntimeError("points are not 3-dimensional")

        # need Nx3 for potential subtraction
        if points.shape[0] == 3:
            points = points.T
        if apply_offset:
            points = numpy.subtract(points, self.origin)
        
        return numpy.dot(self.mat, points.T).T

    # point are 3-dimensional in local frame, result is Nx3
    def convert_points_to_world_frame(self, points, apply_offset = False):
        if points.ndim == 1:
            points = points[numpy.newaxis, :]
        if 3 not in points.shape or points.ndim != 2:
            raise RuntimeError("points are not 3-dimensional")

        if points.shape[0] == 3:            
            points_world = numpy.dot(self.mat.T, points).T
        elif points.shape[1] == 3:            
            points_world = numpy.dot(self.mat.T, points.T).T

        if apply_offset:
            points_world = numpy.subtract(points_world, self.origin)
        return points_world
            
    # convert Rotation matrix in local frame to world frame
    def rotation_matrix_to_world_frame(self, R):
        return numpy.dot(self.mat.T, numpy.dot(R, self.mat))        

    # removes coordinate in dim-dimension of local reference frame
    def zero_component_along_dimension(self, points_world, dim):
        assert(0 <= dim <= 2)
        points_local = self.convert_points_from_world_frame(points_world)
        points_local[:, dim] = 0
        points_world_proj = self.convert_points_to_world_frame(points_local)
        return points_world_proj

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2)
    b, c, d = -axis*math.sin(theta/2)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def angle_between_vectors(v1, v2):
    return numpy.arccos(numpy.dot(v1, v2) / (numpy.linalg.norm(v1) * numpy.linalg.norm(v2)))

# returns centered (N-1) vectors from P_0 -> P_1 -> ... P_N
def get_path_vectors(path):
    path_shift = numpy.zeros((path.shape[0] + 1, path.shape[1]))
    path_shift[1:, :] = path
    path_aug = numpy.vstack((path, numpy.zeros((1, 3))))
    
    vecs = (path_aug - path_shift)[1:-1, :]
    return vecs


# point on plane can be arbitrary, it's just used for calculating
# distance along normal
def project_points_to_plane(p_on_plane, normal, points):
    vecs = points - p_on_plane
    
    normal = normal / numpy.linalg.norm(normal)

    dists = numpy.dot(vecs, normal)

    all_normals = numpy.tile(normal[:, numpy.newaxis], vecs.shape[0])

    proj = points.T - dists * all_normals
    return proj.T

# generats 4 coords by maximizing along x,y axes and solving for z in plane equation
def generate_normal_plane_to_3d_vector_at_loc(vec, loc, extent = 1.0):
    x1 = extent / 2.
    x2 = x1
    x3 = -x1
    x4 = x3

    y1 = extent / 2.
    y2 = -y1
    y3 = y1
    y4 = y2

    z1 = calc_plane_z_from_x_y_vec_and_loc(x1, y1, vec, loc)
    z2 = calc_plane_z_from_x_y_vec_and_loc(x2, y2, vec, loc)
    z3 = calc_plane_z_from_x_y_vec_and_loc(x3, y3, vec, loc)
    z4 = calc_plane_z_from_x_y_vec_and_loc(x4, y4, vec, loc)
    
    mesh_points = numpy.array([[x1, y1, z1],
                               [x2, y2, z2],
                               [x3, y3, z3],
                               [x4, y4, z4]])
    return mesh_points

def calc_plane_z_from_x_y_vec_and_loc(x1, y1, vec, loc):
    a = vec[0]
    b = vec[1]
    c = vec[2]
    x0 = loc[0]
    y0 = loc[1]
    z0 = loc[2]

    return (a * (x0 - x1)) / (c + sys.float_info.epsilon) + (b * (y0 - y1)) / (c + sys.float_info.epsilon) + z0

# plane is now a (4,) array
def point_and_normal_to_plane(point, normal_through_point):
    assert(len(point) == 3)
    assert(len(normal_through_point) == 3)
    d = - numpy.dot(point, normal_through_point)
    ntpl = list(normal_through_point)
    ntpl.append(d)
    return numpy.asarray(ntpl)

def distance_to_plane(point, plane):
    assert(len(plane) == 4)
    assert(len(point) == 3)
    
    d  = plane[0] * point[0] + plane[1]*point[1] + plane[2]*point[2] + plane[3] / \
         float(numpy.linalg.norm(plane[:3]))
    return d


def distances_to_plane(arr_points, plane):
    assert(len(plane) == 4)
    assert(arr_points.shape[1] == 3)

    distances = (numpy.dot(arr_points, plane[:3]) + plane[3]) / numpy.linalg.norm(plane[:3])
    return distances

def plane_mgrid_to_normal(xx, yy, zz):
    midpointx = xx[xx.shape[0]/2][xx.shape[1]/2]
    midpointy = yy[yy.shape[0]/2][yy.shape[1]/2]
    midpointz = zz[zz.shape[0]/2][zz.shape[1]/2]

    p0 = numpy.array([midpointx, midpointy, midpointz])
    p1 = numpy.array([xx[0][1], yy[0][1], zz[0][1]])
    p2 = numpy.array([xx[-1][-1], yy[-1][-1], zz[-1][-1]])
    normal = numpy.cross(p1 - p0, p2 - p0)
    normal /= numpy.linalg.norm(normal)
    return normal

class quaternion(object):
    
    @type_util.member_initializer
    def __init__(self, w, x, y, z):        
        self.normalized = False

    def to_numpy(self):
        return numpy.array([self.w, self.x, self.y, self.z])
        
    def get_conjugate(self):
        if not self.normalized:
            self.normalize()
        return quaternion(self.w, -self.x, -self.y, -self.z)
        
    def normalize(self):
        mag = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
        self.w /= mag
        self.x /= mag
        self.y /= mag
        self.z /= mag
        self.normalized = True

    def __mul__(self, o):
        new_x = self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y
        new_y = self.w * o.y + self.y * o.w + self.z * o.x - self.x * o.z
        new_z = self.w * o.z + self.z * o.w + self.x * o.y - self.y * o.x
        new_w = self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z
        return quaternion(new_w, new_x, new_y, new_z)
        
    def rotate_vector(self, v):
        v = quaternion(0, v[0], v[1], v[2])
        v.normalize()

        quat_res = (self * (v * self.get_conjugate()))
        return numpy.array([quat_res.x, quat_res.y, quat_res.z])
