'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
def printTendril(tendril, n):
    for x in tendril :
        print  n*" " +" name=" + x.key() + " type=%s"%x.data().type_name() + " default=",x.data().get()
        print  (n+2)*" " +x.data().doc()
def printModuleDoc(m):
    print "Module: "+ m.Name()
    print "     Doc: " + m.Doc()
    print "  inputs:"
    printTendril(m.inputs,2)
    print " outputs:"
    printTendril(m.outputs,2)
    print "  params:"
    printTendril(m.params,2)
def graphviz(plasm):
    print "digraph plasm {"
    print plasm.viz()
    print "}"
