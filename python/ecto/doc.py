'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
import ecto, xdot
import gtk
import gtk.gdk
import inspect
from pydoc import ispackage
from inspect import ismodule

def print_tendrils(tendril, n):
    for x in tendril :
        #print "here"
        value = str(x.data().get())
        print  " - " + x.key() + " [%s]"%x.data().type_name + " default = %s"%value
        print  ""
        docstr = str(x.data().doc)
        doclines = docstr.splitlines()
        if doclines :
            for docline in doclines:
                print  "    " + docline
        print  ""

def print_module_doc(m):
    print m.__doc__
    

class PlasmDotView(xdot.DotWindow):
    def __init__(self):
        xdot.DotWindow.__init__(self)


def list_ecto_module(pymodule):
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule,x)
        if inspect.isclass(mod) and issubclass(mod,ecto._module_base):
                m = mod.inspect((),{})
                print_module_doc(m)
                l.append(m)
    return l
    
def view_plasm(plasm):
    window = PlasmDotView()
    x = plasm.viz()
    window.set_dotcode(x)
    window.connect('destroy',gtk.main_quit)
    gtk.main()
    
