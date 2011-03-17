'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
def printModuleDoc(m):
    print "inputs:"
    for x in m.inputs :
        print "\t",x.data().name(),"type=%s"%x.data().type_name()
        print "\t\t", x.data().doc()
    print "outputs:"
    for x in m.outputs :
        print "\t",x.data().name(),"type=%s"%x.data().type_name()
        print "\t\t", x.data().doc()
    print "params:"
    for x in m.params :
        print "\t",x.data().name(),"type=%s"%x.data().type_name()
        print "\t\t", x.data().doc()

def graphviz(plasm):
    print "digraph plasm {"
    print plasm.viz()
    print "}"
