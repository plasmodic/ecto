'''
        Created on Mar 12, 2011
        
        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
import ecto

import pygraphviz as pgv
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
    str = """digraph plasm {
    %s
    }"""%plasm.viz()
    nodes = []
    edges = []
    for el in plasm.edges :
        module =  el.key()
        nodes.append(module.__str__())
        downstream = el.data().downstream
        for d_el in downstream:
            for m_el in d_el.data():
                edges.append((module.__str__() + "_" + d_el.key(),m_el.data()))
    #print nodes
    #print edges
    return str    
    
