from multiprocessing import Process
from collections import OrderedDict
import os, pdb
import time
import socket
import sys
import thread
import subprocess

import type_util as typeu
import data.vsfm_ui as vsfm_ui

cur_dir = os.path.dirname(os.path.realpath(__file__))

# programmatically make functions based on py dictionary of commands
class vsfm_commander(object):
    def __init__(self, socket):
        self.functions = OrderedDict()
        self.socket = socket
        self.create_functions_from_dictionary(vsfm_ui.menu)

    def __repr__(self):
        s = 'VSFM Commander on socket {}. Function List:\n--\n'.format(self.socket.getsockname()[1])
        for fn, (fid, _) in self.functions.items():
            s += '{}(*args, **kwargs) ({})\n'.format(fn, fid)

        return s

    # create functions with prefixes depending on menu / dictionary hierarchy
    def create_functions_from_dictionary(self, d, prefix = ''):
        for k,v in d.items():
            if isinstance(k, str) and k.find('menu') == 0:
                self.create_functions_from_dictionary(v, prefix = '_'.join(k.split('_')[1:]) + '_')
            else:
                fid, func_name = k, prefix + v

                assert(not hasattr(self, func_name))

                func = self.create_single_function(fid, func_name)
                setattr(self, func_name, func)
                self.functions[func_name] = fid, func

    # create functions that send commands over socket... these functions will return after
    # sending the command
    def create_single_function(self, fid, func_name):
        def _(*args, **kwargs):
            if len(args) > 0:
                cmd = '{} {}\n'.format(fid, *args)
            else:
                cmd = '{}\n'.format(fid)
            print "sending command over port {}: ({},{}".format(self.socket.getsockname()[1], func_name, cmd)
            self.socket.sendall(cmd)
        return _                        

# main interface class
class vsfm_interface(object):
    
    @typeu.member_initializer
    def __init__(self, 
                 vsfm_binary_fn = '/home/nrhineha/dev/vsfm/bin/VisualSFM', 
                 port = None,
                 host = 'localhost'):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.port is None:
            tmp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tmp_sock.bind(('', 0))
            self.port = tmp_sock.getsockname()[1]
            tmp_sock.close()
            del tmp_sock
            print "will bind to port: {}".format(self.port)

        self.vsfm_process = Process(target = self.start_program).start()
        
        for _ in range(3):
            try:
                self.sock.connect((self.host, self.port))
                break
            except:
                time.sleep(0.1)

        self.commander = vsfm_commander(self.sock)
        self.add_functions_from_commander()
        self.create_overrides()
        
    def add_functions_from_commander(self):
        for func_name, (fid, func) in self.commander.functions.items():
            setattr(self, func_name, func)

    # create overriding functions... e.g. dense_reconstruction requires a 
    # path but will fail silently if you don't pass one
    def create_overrides(self):
        def reconstruct_dense(path = 'dense_recon'):
            path = os.path.abspath(path)
            if not os.path.isdir(path):
                os.mkdir(path)
            
            fid, rd_func = self.commander.functions['sfm_reconstruct_dense']
            rd_func(path)
        setattr(self, 'sfm_reconstruct_dense', reconstruct_dense)

    def start_program(self):
        self.cmd = '{} listen[+log] {}'.format(self.vsfm_binary_fn, self.port)
        self.args = self.cmd.split(' ')
        self.vsfm_subprocess = subprocess.Popen(self.args)
    
    def close(self):
        self.__del__()

    def __del__(self):
        self.commander.exit_program()
        self.sock.close()

        
