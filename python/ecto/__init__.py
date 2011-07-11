#  Copyright (C) 2008   Troy D. Straszheim  <troy@icecube.umd.edu>
#  and the IceCube Collaboration <http://www.icecube.wisc.edu>
# 
#  This file is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>
# 
"""
"""

import platform, sys

if platform.system().startswith('freebsd'):
        # C++ modules are extremely fragile when loaded with RTLD_LOCAL,
        # which is what Python uses on FreeBSD by default, and maybe other
        # systems. Convince it to use RTLD_GLOBAL.

        # See thread by Abrahams et al:
        # http://mail.python.org/pipermail/python-dev/2002-May/024074.html
        sys.setdlopenflags(0x102)


def load_pybindings(name, path):
    """
    Merges python bindings from shared library 'name' into module 'name'.
    Use when you have a directory structure::

      lib/
         foo.so
         foo/
           __init__.py
           something.py

    Here, inside ``foo/__init__.py`` call ``load_pybindings(__name__, __path__)``

    this assumes that the first entry in list ``__path__`` is where
    you want the wrapped classes to merge to.

    """

    import imp, sys
    m = imp.load_dynamic(name, path[0] + ".so")
    thismod = sys.modules[name]

    for (k,v) in m.__dict__.items():
        if not k.startswith("_"):
            thismod.__dict__[k] = v

print ">>> NAME=", __name__, "<<<"
print ">>> PATH=", __path__, "<<<"

load_pybindings(__name__, __path__)

from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)

from doc import *
from module import *
from blackbox import *
