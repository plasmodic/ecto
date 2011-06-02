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
        print  " - " + x.key() + " [%s]"%x.data().type_name + " default =",x.data().val

        print  ""
        print  "    " + x.data().doc
        print  ""

def print_module_doc(m):
    print m.name(), "(ecto::module)"
    print "================================="
    print ""
    print m.doc()
    print ""
    if len(m.params):
        print "params"
        print "---------------------------------"
        print ""
        print_tendrils(m.params,4)
    if len(m.inputs):
        print "inputs"
        print "---------------------------------"
        print ""
        print_tendrils(m.inputs,4)
    if len(m.outputs):
        print "outputs"
        print "---------------------------------"
        print ""
        print_tendrils(m.outputs,4)
    print ""
    

class PlasmDotView(xdot.DotWindow):
    def __init__(self):
        xdot.DotWindow.__init__(self)


def list_ecto_module(pymodule):
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule,x)
        #if ismodule(mod):
        #    list_ecto_module(mod)
        if inspect.isclass(mod) and issubclass(mod,ecto._module_base):
            try:
                m = mod.inspect((),{})
                print_module_doc(m)
                l.append(m)
            except Exception,e:
                print e
    return l
    
def view_plasm(plasm):
    window = PlasmDotView()
    x = plasm.viz()
    window.set_dotcode(x)
    window.connect('destroy',gtk.main_quit)
    gtk.main()
    
