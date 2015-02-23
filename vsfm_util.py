import vsfm_socket_util as vsfmu
import os, time, glob, ipdb, pdb, numpy, uuid, signal, gtk
import inspect, threading

import type_util, math_util as mathu
import mayavi_util as mayaviu
import mayavi.mlab as mlab
from pyface.api import GUI


import util
import argh
import sklearn.neighbors.kd_tree

# import gtkutils.color_printer

class vsfm_camera(object):
    @type_util.member_initializer
    def __init__(self, filename, focal_length, qx, qy, qz, qw, cx, cy, cz, radial_dist, zero):
        self.camera_position = map(float, [cx, cy, cz])
        self.quaternion = mathu.quaternion(*map(float, [qw, qx, qy, qz]))
        self.new = False
        
    def get_orientation_vec(self):
        return self.quaternion.rotate_vector([0., 0., -1.])

class vsfm_point(object):
    @type_util.member_initializer
    def __init__(self, x, y, z, r, g, b):
        self.position = map(float, [x, y, z])
        self.color = map(float, [r, g, b])
        self.color_norm = [self.color[0] / 255., self.color[1] / 255., self.color[2] / 255.]

class vsfm_model(object):
    @type_util.member_initializer
    def __init__(self, cameras, points):
        camera_positions = numpy.array([c.camera_position for c in cameras])
        self.camera_tree = sklearn.neighbors.kd_tree.KDTree(camera_positions)

    @staticmethod
    def create_from_nvm(nvm_file):
        models, model_points = extract_cameras_and_points_from_nvm(nvm_file)
        cams, points = models[0], model_points[0]
        return vsfm_model(cams, points)

    def lookup_nearby_cameras(self, camera, radius = 2.0):
        p = camera.camera_position
        nearby_camera_inds = self.camera_tree.query_radius(p, radius, return_distance = False)[0]
        nearby_cameras = [self.cameras[_] for _ in nearby_camera_inds]
        return nearby_cameras

class VSFMCrashedError(Exception):
    pass

class register_image_threaded(threading.Thread):

    @type_util.member_initializer
    def __init__(self, vsfm_interface, imfn, max_sleep_seconds, match_specified_fn = None):
        threading.Thread.__init__(self)

    def run(self):
        cams, new_cams = register_image(self.vsfm_interface, self.imfn, 
                                        match_specified_fn = self.match_specified_fn,
                                        max_sleep_seconds = self.max_sleep_seconds)
        GUI.invoke_later(mayaviu.plot_cameras, new_cams)
        self.cams = cams
        self.new_cams = new_cams

class rts_thread(threading.Thread):
    @type_util.member_initializer
    def __init__(self, vsfm_interface, nvm_file, model, image_files, max_sleep_seconds, model_image_dir = '/home/nrhineha/dev/activity/data/kitchen_seq_x1'):
        threading.Thread.__init__(self)

    def run(self):
        register_temporal_sequence(self.vsfm_interface, self.nvm_file, self.model, 
                                   model_image_dir = self.model_image_dir,
                                   image_files = self.image_files,
                                   max_sleep_seconds = self.max_sleep_seconds)



        
def write_specified_match_file(new_cam_filename, match_cams, match_cams_dir):
    mfn = 'tmp_specified_match.txt'
    with open('tmp_specified_match.txt', 'w') as f:
        for m in match_cams:
            f.write('{} {}/{}\n'.format(new_cam_filename, match_cams_dir, m.filename))
    return mfn

def images_in_path(images_path, ext = '.jpg'):
    image_files = sorted([os.path.abspath('{}/{}'.format(images_path, i)) \
                          for i in os.listdir(images_path) if i.find(ext) == len(i) - len(ext)])
    return image_files

def localized_image_signal(signum, frame):
    frame_locals = inspect.getargvalues(frame)[-1]
    cam = frame_locals['new_cams'][0]
    cam_pos = cam.camera_position
    pdb.set_trace()

def plot_localized_image_after_signal(fig):
    def func(sig, frame):
        frame_locals = inspect.getargvalues(frame)[-1]
        cam = frame_locals['new_cams'][0]
        cam_pos = cam.camera_position
        mayaviu.plot_cameras([cam], fig)
    return func

# def load_nvm_and_register_images(nvm_file, 
#                                  images_path = None, 
#                                  every_nth = 1,
#                                  single_image = None,
#                                  max_sleep_seconds = 15):

def load_nvm_and_register_images(nvm_file, 
                                 images_path, 
                                 max_sleep_seconds,
                                 every_nth,
                                 single_image = None):

    cams, points, cams_mlab, points_mlab = mayaviu.load_and_plot_nvm_cams(nvm_file)    
    fig = mlab.gcf()
    # signal.signal(signal.SIGUSR1, plot_localized_image_after_signal(fig))

    # if single_image is not None:
    #     tmp_dir = 'tmp_im_dir'
    #     if not os.path.isdir(tmp_dir):
    #         os.mkdir(tmp_dir)
    #     shutil.copy(
    
    # signal.signal(signal.SIGUSR1, localized_image_signal)

    model = vsfm_model.create_from_nvm(nvm_file)
    vsfm_interface = vsfmu.vsfm_interface()
    vsfm_interface.sfm_more_less_visualization_data()
    
    image_files = ['{}/{}'.format(images_path, i) \
                      for i in os.listdir(images_path) if i.find('.jpg') > 0]
    image_basenames = sorted([os.path.basename(_) for _ in image_files])
    image_files = sorted([os.path.abspath(_) for _ in image_files])

    vsfm_interface.sfm_load_nview_match(nvm_file)
    vsfm_interface.view_image_thumbnails()

    mlab.show()
    rtst = rts_thread(vsfm_interface, 
                      nvm_file,
                      model, 
                      model_image_dir = '/home/nrhineha/dev/activity/data/kitchen_seq_x1',
                      image_files = image_files,
                      max_sleep_seconds = max_sleep_seconds)
    rtst.start()
    mlab.show()
    
    # seq = register_temporal_sequence(vsfm_interface, 
    #                                  nvm_file,
    #                                  model, 
    #                                  model_image_dir = '/home/nrhineha/dev/activity/data/kitchen_seq_x1',
    #                                  image_files = image_files,
    #                                  max_sleep_seconds = max_sleep_seconds)
    
    # cams, new_cams = register_image(vsfm_interface, image_files[0])

    # # vsfm_interface.sfm_delete_selected_camera()

    # near_cams= model.lookup_nearby_cameras(new_cams[0])
    # mfn = write_specified_match_file('/home/nrhineha/Desktop/test/loc2.jpg',
    #                                  near_cams, 
    #                                  '/home/nrhineha/dev/activity/data/kitchen_seq_x1')

    # register_image(vsfm_interface, images_path, image_files[1], image_basenames[1], match_specified_fn = os.path.abspath(mfn))
    
    ipdb.set_trace()

def load_nvm_and_register_image(nvm_file, image_path, max_sleep_seconds = 30):
    # with gtkutils.color_printer.timer() as t:
        vsfm_interface = vsfmu.vsfm_interface()
        vsfm_interface.sfm_more_less_visualization_data()
        vsfm_interface.sfm_load_nview_match(nvm_file)                               
        cams, new_cams = register_image(vsfm_interface, image_path, max_sleep_seconds = max_sleep_seconds)
        if len(new_cams) > 0:
            print "camera position: {}".format(new_cams[0].camera_position)
        else:
            print "couldn't localize!"



def register_temporal_sequence(vsfm_interface, nvm_model_fn, model, model_image_dir, image_files, radius = 2, max_sleep_seconds = 15):

    cams = []
    new_cams = []
    near_cams = []
    sequence = []

    def reset_model():
        print "clearing workspace & reloading model"
        vsfm_interface.restart()
        vsfm_interface.sfm_clear_workspace()
        vsfm_interface.sfm_load_nview_match(nvm_model_fn)

    seq_positions = numpy.nan * numpy.ones((len(image_files), 3), dtype = numpy.float64)
    query_cam = None
    mlab.show()

    colors1 = numpy.linspace(0, 1, len(image_files))
    colors2 = numpy.tile(numpy.linspace(0, 1, len(image_files)/2), 2)
    colors3 = numpy.tile(numpy.linspace(0, 1, len(image_files)/4), 4)
    
    while colors2.shape[0] < colors1.shape[0]:
        colors2 = numpy.hstack((colors2, 0))
    while colors3.shape[0] < colors1.shape[0]:
        colors3 = numpy.hstack((colors3, 0))
    colors = numpy.vstack((colors1, colors2, colors3)).T
    
    for (im_idx, imfn) in enumerate(image_files):
        imdir, imbasename, ext = util.fileparts(imfn)

        failed = False
        if len(near_cams) == 0:
            try:
                cams, new_cams = register_image(vsfm_interface, imfn, max_sleep_seconds = max_sleep_seconds)
            except VSFMCrashedError:
                reset_model()
        else:
            mfn = write_specified_match_file(imfn, near_cams, model_image_dir)

            try:
                cams, new_cams = register_image(vsfm_interface, imfn, match_specified_fn = mfn, max_sleep_seconds = max_sleep_seconds)
            except VSFMCrashedError:
                reset_model()

        if len(new_cams) == 0:
            print "OH NO, Didn't localize!!!"
            failed = True
            if len(near_cams) > 0:
                print "trying more image matching"

                for r in [radius * 2, radius * 4, radius * 1e100]:
                    near_cams_2 = model.lookup_nearby_cameras(query_cam, radius = r)
                    mfn = write_specified_match_file(imfn, near_cams_2, model_image_dir)

                    try:
                        cams, new_cams = register_image(vsfm_interface, imfn, match_specified_fn = mfn,
                                                        rerun_sift = True, max_sleep_seconds = max_sleep_seconds)
                    except VSFMCrashedError:
                        reset_model()

                    if len(new_cams) == 0:
                        print "Matching with radius failed: {}".format(r)
                    else:
                        print "Matching success!"
                        failed = False
                        break
            else:
                print "already matched against all cams, localization will not work"

            if failed:
                reset_model()
                sequence.append(None)
                continue
        else:
            print "\nLocalized new cam! {}".format(new_cams[0].camera_position)
            GUI.invoke_later(mayaviu.plot_cameras, new_cams, color = tuple(colors[im_idx, :]))
            # os.kill(os.getpid(), signal.SIGUSR1)

        query_cam = new_cams[0]
        near_cams = model.lookup_nearby_cameras(query_cam, radius = radius)
        sequence.append(new_cams[0])
        
        if new_cams[0] is not None:
            seq_positions[im_idx, :] = new_cams[0].camera_position
                                
        # vsfm_interface.sfm_delete_selected_camera()
        vsfm_interface.sfm_delete_selected_camera()
        numpy.savez_compressed("cams_localized.npz", seq_positions)
    
    return sequence

def register_image(vsfm_interface, 
                   image_file, 
                   match_specified_fn = None, 
                   max_sleep_seconds = 15, 
                   rerun_sift = False):
    max_sleep_seconds = int(max_sleep_seconds)

    print "IM FN", image_file
    vsfm_interface.file_open_image_and_sift(image_file)



    image_file_dir = os.path.abspath(os.path.dirname(image_file))
    image_basename = os.path.basename(image_file).split('.')[-2]

    image_sift = '{}/{}.sift'.format(image_file_dir, image_basename)
    if not os.path.isfile(image_sift):
        print "running sift"
        vsfm_interface.file_detect_features()
    elif os.stat(image_sift).st_size < 1000 or rerun_sift:
        print "rerunning sift"
        os.remove(image_sift)
        vsfm_interface.file_detect_features()
        

    print "registering image: {}, image path: {}, basename: {}".format(image_file, 
                                                                       image_file_dir, 
                                                                       image_basename)

    if match_specified_fn is not None and os.path.isfile(match_specified_fn):
        match_specified_fn = os.path.abspath(match_specified_fn)
        vsfm_interface.sfm_pairwise_compute_specified_match(match_specified_fn)
    else:
        vsfm_interface.sfm_pairwise_compute_missing_match()
    
    vsfm_interface.sfm_reconstruct_resume(shift = True)
    # ipdb.set_trace()
    tmp_nvm = '{}/tmp_nvm_{}.nvm'.format(image_file_dir, str(uuid.uuid4()))
    if os.path.isfile(tmp_nvm):
        os.remove(tmp_nvm)

    vsfm_interface.sfm_save_nview_match(tmp_nvm)

    n_sleeps = 0
    sleep_x = .1

    while not os.path.isfile(tmp_nvm):
        if n_sleeps > 1 and (n_sleeps % 50 == 0):
            print "Slept {:.1f} / {:.1f} max seconds".format(n_sleeps * sleep_x, max_sleep_seconds)
        time.sleep(sleep_x)
        n_sleeps += 1
        sleep_seconds = sleep_x * n_sleeps
        if sleep_seconds > max_sleep_seconds:
            print "max sleep exceeded! restarting VSFM"
            raise VSFMCrashedError()
        
    models = extract_cameras_from_nvm(tmp_nvm, [image_basename])
    
    if len(models) == 0:
        print "Model extraction failed!!!"
        return [], []
        
    cams = models[0]
    new_cams = []
    for (c_idx, c) in enumerate(cams):
        if c.new:
            new_cams.append(c)
    return cams, new_cams

def register_images(vsfm_interface, image_file_dir, image_files, image_basenames, match_specified_fn = None):
    vsfm_interface.file_open_multi_image(image_files)
    
    if match_specified_fn is not None and os.path.isfile(match_specified_fn):
        match_specified_fn = os.path.abspath(match_specified_fn)
        vsfm_interface.sfm_pairwise_compute_specified_match(match_specified_fn)
    else:
        vsfm_interface.sfm_pairwise_compute_missing_match()
    
    vsfm_interface.sfm_reconstruct_resume(shift = True)
    tmp_nvm = '{}/tmp_nvm_{}.nvm'.format(image_file_dir, str(uuid.uuid4()))
    if os.path.isfile(tmp_nvm):
        os.remove(tmp_nvm)

    vsfm_interface.sfm_save_nview_match(tmp_nvm)

    models = extract_cameras_from_nvm(tmp_nvm, [image_basenames])
    
    if len(models) == 0:
        print "Model extraction failed!!!"
        return [], []
        
    cams = models[0]
    new_cams = []
    for (c_idx, c) in enumerate(cams):
        if c.new:
            new_cams.append(c)
    return cams, new_cams


#
def extract_specific_camera_from_nvm(nvm_file, camera_file_str):
    cams = []
    cam_start = 2
    cam_sec = False
    with open(nvm_file) as f:
        for (idx, line) in enumerate(f):
            if idx < cam_start:
                continue
            if idx == cam_start:
                num_cams = int(line.strip())
                cam_sec = True
                cams = []
                continue
            if cam_sec:
                if idx - cam_start > num_cams:
                    cam_sec = False
                    return None
                x = line.strip().replace('\t', ' ').split(' ')
                if x[0].find(camera_file_str) >= 0:
                    return vsfm_camera(*x)
    return None


#only parses cameras of first model
def extract_cameras_from_nvm(nvm_file, new_flag_strs = []):
    models = []
    cams = []
    cam_start = 2
    cam_sec = False
    point_sec = False
    with open(nvm_file) as f:
        for (idx, line) in enumerate(f):
            if idx < cam_start:
                continue
            if idx == cam_start:
                num_cams = int(line.strip())
                cam_sec = True
                cams = []
                continue
            if cam_sec:
                if idx - cam_start > num_cams:
                    cam_sec = False
                    models.append(cams)
                    break
                x = line.strip().replace('\t', ' ').split(' ')
                cam = vsfm_camera(*x)
                if len(new_flag_strs) > 0:
                    for (nfg_idx, nfs) in enumerate(new_flag_strs):
                        if x[0].find(nfs) >= 0:
                            cam.new= True

                            #careful, probably not removing non-inorder things correctly
                            new_flag_strs.pop(nfg_idx)

                cams.append(cam)
    return models

#only parses cameras and points of first model
def extract_cameras_and_points_from_nvm(nvm_file):
    models = []
    cams = []
    points = []
    cam_start = 2
    cam_sec = False
    point_sec = False
    n_points = -1
    with open(nvm_file) as f:
        for (idx, line) in enumerate(f):

            if idx < cam_start:
                continue
            if idx == cam_start:
                num_cams = int(line.strip())
                cam_sec = True
                cams = []
                continue
            if cam_sec:
                if idx - cam_start > num_cams:
                    cam_sec = False
                    point_sec = True
                    models.append(cams)
                    continue

                x = line.strip().replace('\t', ' ').split(' ')
                cam = vsfm_camera(*x)
                cams.append(cam)

            if point_sec:
                line_strip = line.strip()
                if 0 <= len(line_strip) < 6:
                    if n_points == -1:
                       n_points = int(line_strip)
                       continue
                    else:
                        break
                elif len(points) == n_points:
                    break
                else:
                    point = vsfm_point(*line_strip.split(' ')[:6])
                    points.append(point)
    
    for (cams_idx, cams) in enumerate(models):
        cams.sort(key = lambda c: c.filename)
    return models, points

def sparse_reconstruction_from_image_dir(path):
    cur_dir = os.getcwd()
    bn = os.path.basename(path)
    print "basename", bn
    os.chdir(path)

    i = vsfmu.vsfm_interface()

    i.file_open_current_path()
    i.sfm_pairwise_compute_missing_match()
    i.sfm_reconstruct_sparse()

    i.sfm_save_nview_match(cur_dir + '/{}_sparse.nvm'.format(bn))

    pdb.set_trace()

###
def write_matches_list(l, fn):
    with open(fn, 'w') as f:
        for i1, i2 in l:
            f.write('{} {}\n'.format(i1, i2))

###
def write_ims_list(l, fn):
    with open(fn, 'w') as f:
        for i in l:
            f.write('{}\n'.format(i))

def all_pairwise_matches_list(ims):
    l = []
    for (i1, im1) in enumerate(ims):
        for im2 in ims[i1 + 1:]:
            l.append((im1, im2))
    return l

@argh.set_all_toggleable()
def create_pairwise_matching_file_and_ims_list(path_1, path_2, 
                                               first_start = 1, first_end = 100, 
                                               second_start = 1, second_end = 100,
                                               no_list_1 = False):
    n_first = first_end - first_start
    n_second = second_end - second_start
    assert(n_first > 0 and n_second > 0)

    path_1 = os.path.abspath(path_1)
    path_2 = os.path.abspath(path_2)

    path_1_ims = images_in_path(path_1)
    path_2_ims = images_in_path(path_2)

    assert(first_end <= len(path_1_ims))
    assert(second_end <= len(path_2_ims))

    if len(path_1_ims) == 0 or len(path_2_ims) == 0:
        raise RuntimeError("empty dirs!")

    all_ims = path_1_ims + path_2_ims

    data_source_1 = path_1.split('/')[-2]
    data_source_2 = path_2.split('/')[-2]

    if no_list_1:
        l1 = []
    else:
        l1 = all_pairwise_matches_list(path_1_ims)

    l2 = all_pairwise_matches_list(path_2_ims)
    print "n_path_1: {} -> {}".format(len(path_1_ims), len(l1))
    print "n_path_2: {} -> {}".format(len(path_2_ims), len(l2))

    print "n_path_1[s:e] = {}".format(len(path_1_ims[first_start:first_end]))
    
    l1.extend(l2)


    for (i1, im1) in enumerate(path_1_ims[first_start:first_end]):
        for (i2, im2) in enumerate(path_2_ims[second_start:second_end]):
            l1.append((im1, im2))

    print "final n matches: {}".format(len(l1))
    write_matches_list(l1, 'two_dir_matches_{}x{}_n1-{:d}to{:d}_n2-{:d}to{:d}_ntot{}.txt'.format(data_source_1,
                                                                                               data_source_2,
                                                                                               first_start, first_end,
                                                                                               second_start,second_end,
                                                                                               len(l1)))
    write_ims_list(all_ims, 'two_dir_ims_{}x{}_ntot{}.txt'.format(data_source_1,
                                                                  data_source_2,
                                                                  len(all_ims)))

if __name__ == '__main__':
    parser = argh.ArghParser()
    parser.add_commands([load_nvm_and_register_images, 
                         load_nvm_and_register_image,
                         sparse_reconstruction_from_image_dir,
                         create_pairwise_matching_file_and_ims_list])
    util.ipdbwrap(parser.dispatch)()
