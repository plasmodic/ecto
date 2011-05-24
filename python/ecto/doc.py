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
        print  n*" " + x.key() + " [%s]"%x.data().type_name + " default =",x.data().val
        print  (n+2)*" " +x.data().doc

def print_module_doc(m):
    print "================================="
    print m.name(), "(ecto::module)"
    print 4*" " + m.doc()
    if len(m.params):
        print "---------------------------------"
        print "  params:"
        print_tendrils(m.params,4)
    if len(m.inputs):
        print "---------------------------------"
        print "  inputs:"
        print_tendrils(m.inputs,4)
    if len(m.outputs):
        print "---------------------------------"
        print " outputs:"
        print_tendrils(m.outputs,4)
    print "---------------------------------"

    

class PlasmDotView(xdot.DotWindow):
    def __init__(self):
        xdot.DotWindow.__init__(self)


def list_ecto_module(pymodule):
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule,x)
        if ismodule(mod):
            list_ecto_module(mod)
        if inspect.isclass(mod) and issubclass(mod,ecto._module_base):
            m = mod()
            print_module_doc(m)
            l.append(m)
    return l
    
def view_plasm(plasm):
    window = PlasmDotView()
    x = plasm.viz()
    window.set_dotcode(x)
    window.connect('destroy',gtk.main_quit)
    gtk.main()
    
