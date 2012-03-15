"""
Function meant to help
http://comments.gmane.org/gmane.comp.python.c%2B%2B/13004
"""

import os
import platform
import sys

if platform.system().startswith('freebsd'):
        # C++ modules are extremely fragile when loaded with RTLD_LOCAL,
        # which is what Python uses on FreeBSD by default, and maybe other
        # systems. Convince it to use RTLD_GLOBAL.

        # See thread by Abrahams et al:
        # http://mail.python.org/pipermail/python-dev/2002-May/024074.html
        sys.setdlopenflags(0x102)

def load_pybindings(name, input_path=None):
    """
    Has to be called from an __init__.py as follows:
    from object_recognition_core.utils.load_pybindings import load_pybindings
    __path__ = load_pybindings(__name__)

    If the input_path is provided, it will assume such structure (otherwise, it looks for foo.so anymore in the
    PYTHONPATH)

      lib/
         foo.so
         foo/
           __init__.py
           something.py

    Here, inside ``foo/__init__.py`` call ``load_pybindings(__name__, __path__)``

    this assumes that the first entry in list ``__path__`` is where
    you want the wrapped classes to merge to.
    """
    thismod = sys.modules[name]
    for path in os.environ['PYTHONPATH'].split(':'):
        if input_path:
            potential_additional_path = input_path
        else:
            potential_additional_path = os.path.join(path, *(name.split('.')))

        if os.path.isdir(potential_additional_path):
            if potential_additional_path not in thismod.__path__:
                thismod.__path__.append(potential_additional_path)

        potential_additional_file = potential_additional_path + ".so"

        if not os.path.isfile(potential_additional_file):
            continue

        import imp
        m = imp.load_dynamic(name, potential_additional_file) #TODO this is only going to work on unix...

        for (k, v) in m.__dict__.items():
            if not k.startswith("_"):
                thismod.__dict__[k] = v

        if input_path:
            break
