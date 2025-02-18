import sys
import traceback



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
