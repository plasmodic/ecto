'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
import ecto
import inspect
from pydoc import ispackage
from inspect import ismodule

def print_tendrils(tendril, n):
    for x in tendril :
        #print "here"
        value = str(x.data().get())
        print  " - " + x.key() + " [%s]" % x.data().type_name + " default = %s" % value
        print  ""
        docstr = str(x.data().doc)
        doclines = docstr.splitlines()
        if doclines :
            for docline in doclines:
                print  "    " + docline
        print  ""

def print_module_doc(m):
    print m.__doc__
    

def list_all_ecto_modules(pymodule):
    '''
    Creates a list of all cells from a python module, which are ready for doc string and other
    types of introspection.
    '''
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule, x)
        if inspect.isclass(mod) and issubclass(mod, ecto._cell_base):
                m = mod.inspect((), {})
                l.append(m)
    return l

def list_ecto_module(pymodule):
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule, x)
        if inspect.isclass(mod) and issubclass(mod, ecto._cell_base):
                m = mod.inspect((), {})
                if m.__doc__ != None :
                    print m.__doc__
                else :
                    raise ValueError("A module's doc string may not be None.")
                l.append(m)
    return l

def view_plasm(plasm):
    try:
        import gtk
        import xdot
    except ImportError, e:
        print e
        print "view_plasm requires gobject gtk graphviz, possibly more to run..."
        return
    window = xdot.DotWindow()
    x = plasm.viz()
    window.set_dotcode(x)
    window.connect('destroy', gtk.main_quit)
    gtk.main()
