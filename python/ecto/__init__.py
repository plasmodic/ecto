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

import platform, sys 

#import inspect

class EctoCellBase(object):
    pass

def cell_getitem(self, *args, **kwargs):
    print "GETITEM!", self, args, kwargs
    if len(args) == 1 and type(args[0]) == slice:
        return __getitem_slice__(self.__impl, args[0])
    if len(args) == 1 and type(args[0]) == tuple:
        return __getitem_tuple__(self.__impl, args[0])
    if len(args) == 1 and type(args[0]) == list:
        return __getitem_list__(self.__impl, args[0])

    return __getitem_str__(self.__impl, args[0])

def cellinit(cpptype):
    def impl(self, *args, **kwargs):
        if len(args) > 1:
            raise RuntimeError("Too many positional args:  only one allowed, representing cell instance name")
        e = lookup(cpptype)
        c = self.__impl = e.construct()
        if len(args) == 1:
            self.__impl.name(args[0])
        e.declare_params(self.__impl.params)
        # c.construct(args, kwargs)
        #print "c=", c
        self.inputs = c.inputs
        self.outputs = c.outputs
        self.params = c.params
        for k, v in kwargs.iteritems():
            if k == 'strand':
                self.__impl._set_strand(v)
            else:
                setattr(self.params, k, v)
            # print "now:", getattr(self.params, k)
        e.declare_io(self.params, self.inputs, self.outputs)
        self.__impl.verify_params()
    # self.params.get('k') = v
    return impl

def cell_print_tendrils(tendril):
    s = ""
    for x in tendril:
        print ">>>>>", x.data().doc
        try:
            value = str(x.data().get())
        except TypeError, e:
            value = "[unprintable]"
        s += " - " + x.key() + " [%s]" % x.data().type_name
        if x.data().required:
            s += " REQUIRED"

        if x.data().has_default:
            s += " default = " + value
        s += "\n"
        docstr = str(x.data().doc)
        doclines = docstr.splitlines()
        if doclines :
            for docline in doclines:
                s +=  "    " + docline + "\n"
        s +=  "\n"
    return s

@classmethod
def cell_inspect(self, *args, **kwargs):
    # print "cell_inspect!", self, args, kwargs
    c = self()
    return c.__impl

def cell_process(self):
    return self.__impl.process()

def cell_configure(self):
    return self.__impl.configure()

def cell_name(self):
    return self.__impl.name()

def cell_typename(self):
    return self.__impl.typename()

def postregister(cellname, cpptypename, docstring, inmodule):
    #print "POSTREGISTER OF", cellname, "(", cpptypename, ") in", inmodule
    #print "doc:", docstring

    e = lookup(cpptypename)
    c = e.construct()
    c.declare_params()
    c.declare_io()
    #print "ding!", c, c.typename()
    #print c.inputs, c.outputs, c.params
    thistype = type(cellname, (_cell_cpp,), 
                    dict(__doc__ = docstring + "\n\n" 
                         + "Parameters:\n" + cell_print_tendrils(c.params)
                         + "Inputs:\n"+ cell_print_tendrils(c.inputs) 
                         + "Outputs:\n" + cell_print_tendrils(c.outputs), 
                         inputs = c.inputs,
                         outputs = c.outputs,
                         params = c.params,
                         type = c.typename,
                         __init__ = cellinit(cpptypename),
                         __getitem__ = cell_getitem,
                         inspect = cell_inspect,
                         process = cell_process,
                         configure = cell_configure,
                         name = cell_name,
                         type_name = cell_typename,
                         ))

    inmodule.__dict__[cellname] = thistype
    

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

load_pybindings(__name__, __path__)

from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)

from doc import *
from cell import *
from blackbox import *



