'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
import ecto, xdot
import gtk
import gtk.gdk

def print_tendrils(tendril, n):
    for x in tendril :
        print  n*" " + x.key() + " [%s]"%x.data().type_name + " value=",x.data().val
        print  (n+2)*" " +x.data().doc

def print_module_doc(m):
    print "Module: "+ m.name()
    print "     doc: " + m.doc()
    print "  inputs:"
    print_tendrils(m.inputs,2)
    print " outputs:"
    print_tendrils(m.outputs,2)
    print "  params:"
    print_tendrils(m.params,2)
    

class PlasmDotView(xdot.DotWindow):
    def __init__(self):
        xdot.DotWindow.__init__(self)

def view_plasm(plasm):
    window = PlasmDotView()
    window.set_dotcode(plasm.viz())
    window.connect('destroy',gtk.main_quit)
    gtk.main()
    
