import ipdb, os, sys, traceback

def ipdbwrap(f):
    '''A utility for dropping out to a debugger on exceptions.'''
    def fdebug(*a, **kw):
        try:
            return f(*a, **kw)
        except Exception:
            print 
            type, value, tb = sys.exc_info()
            traceback.print_exc(file=sys.stderr)
            os.system('stty sane')
            
            if sys.stdin.isatty():
                ipdb.post_mortem(tb)
            else:
                sys.exit(1)
    return fdebug

#should be same as MATLAB fileparts function
# e.g.
# fileparts('/home/foo/bar.txt') -> ('/home/foo', 'bar', 'txt')
def fileparts(p):
    if p is None:
        return None, None, None

    ap = os.path.abspath(p)
    
    #error check only works if ap actually exists...
    if os.path.isdir(ap):
        return ap + '/', '', ''

    basename = os.path.basename(ap)
    bsplit = basename.split('.')
    if len(bsplit) == 1:
        ext = ''
        name = bsplit[0]
    else:
        ext = bsplit[-1]
        name = basename[:-(len(ext) + 1)]

    loc = os.path.dirname(ap)
    return loc, name, ext
