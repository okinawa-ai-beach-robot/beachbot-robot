import os
import sys
import traceback

class RedirectStdStreams(object):
    def __init__(self, stdout=None, stderr=None):
        self._stdout = stdout or sys.stdout
        self._stderr = stderr or sys.stderr

    def __enter__(self):
        self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
        self.old_stdout.flush(); self.old_stderr.flush()
        sys.stdout, sys.stderr = self._stdout, self._stderr

    def __exit__(self, exc_type, exc_value, traceback):
        self._stdout.flush(); self._stderr.flush()
        sys.stdout = self.old_stdout
        sys.stderr = self.old_stderr

class MonitoredStream():
    def __init__(self, outstream, callback):
        self.outstream = outstream
        self.callback = callback

    def write(self, s):
        self.callback(s)
        self.outstream.write(s)

    def __getattr__(self, attr):
        return getattr(self.outstream, attr)
        
class MonitoredStdStreams(RedirectStdStreams):
    def __init__(self, stdout_callback=None, stderr_callback=None):
        
        self.stdout_callback = stdout_callback
        self.stderr_callback = stderr_callback
        def _print_stdout(s):
            _old_stdout = sys.stdout
            sys.stdout = self.old_stdout
            self.stdout_callback(s)
            sys.stdout = _old_stdout
        def _print_stderr(s):
            _old_stderr = sys.stderr
            sys.stderr = self.old_stderr
            self.stderr_callback(s)
            sys.stderr = _old_stderr
        if stdout_callback is not None:
            self.stdout_monitor = MonitoredStream(sys.stdout, _print_stdout)
        else:
            self.stdout_monitor = None
        if stderr_callback is not None:
            self.stderr_monitor = MonitoredStream(sys.stderr, _print_stderr)
        else:
            self.stderr_monitor = None 
        

        super().__init__(self.stdout_monitor, self.stderr_monitor)

    


def import_class(import_str):
    mod_str, _sep, class_str = import_str.rpartition('.')
    __import__(mod_str)
    try:
        return getattr(sys.modules[mod_str], class_str)
    except AttributeError:
        raise ImportError('Class %s cannot be found (%s)' % (class_str, traceback.format_exception(*sys.exc_info())))

def import_function(import_str):
    mod_str, _sep, func_str = import_str.rpartition('.')
    __import__(mod_str)
    try:
        return getattr(sys.modules[mod_str], func_str)
    except AttributeError:
        raise ImportError('Function %s cannot be found (%s)' % (func_str, traceback.format_exception(*sys.exc_info())))



def instantiate_class(class_name, cargs=[]):
    ClassRef = import_class(class_name)

    if isinstance(cargs, dict):
        class_obj = ClassRef(**cargs)
    elif isinstance(cargs, list):
        class_obj = ClassRef(*cargs)
    else:
        class_obj = ClassRef(cargs)

    return class_obj

def full_class_name(o):
    klass = o.__class__
    module = klass.__module__
    if module == 'builtins':
        return klass.__qualname__ # avoid outputs like 'builtins.str'
    return module + '.' + klass.__qualname__
