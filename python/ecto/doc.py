'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
import ecto, xdot
import gtk
import gtk.gdk

def print_tendrils(tendril, n):
    for x in tendril :
        print  n*" " + x.key() + " type=%s"%x.data().type_name() + " value=",x.data().get()
        print  (n+2)*" " +x.data().doc()
def print_module_doc(m):
    print "Module: "+ m.Name()
    print "     Doc: " + m.Doc()
    print "  inputs:"
    print_tendrils(m.inputs,2)
    print " outputs:"
    print_tendrils(m.outputs,2)
    print "  params:"
    print_tendrils(m.params,2)
    
def graphviz(plasm):
    str = plasm.viz()
    return str  

class PlasmDotView(xdot.DotWindow):

    def __init__(self):
        xdot.DotWindow.__init__(self)
        self.widget.connect('clicked', self.on_url_clicked)

    def on_url_clicked(self, widget, url, event):
        dialog = gtk.MessageDialog(
                parent = self, 
                buttons = gtk.BUTTONS_OK,
                message_format="%s clicked" % url)
        dialog.connect('response', lambda dialog, response: dialog.destroy())
        dialog.run()
        return True
    
def view_plasm(plasm):
    window = PlasmDotView()
    window.set_dotcode(graphviz(plasm))
    window.connect('destroy',gtk.main_quit)
    gtk.main()
    