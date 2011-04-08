#!/usr/bin/env python

import ecto, buster
import test_to_module
strs = {ecto.vertex_t.output : "OUTPUT", ecto.vertex_t.input:"INPUT",ecto.vertex_t.root:"ROOT"}

def print_vertex(vert):
    return vert[0].Name() + ":"+strs[vert[1]]+"(" + vert[2]+")"

def test_iterplasm():
    plasm = test_to_module.makeplasm()
     
    vertices = plasm.vertices()
    edges = plasm.edges()
    
    for s, t in edges:
        source = vertices[s]
        target = vertices[t]   
        print print_vertex(source), " => ", print_vertex(target)
    
    for k,(x,t,name,tendril) in vertices.items():
        if(t == ecto.vertex_t.root):
            print x.Name(), " has parameters:"
            for p in x.params:
                print p.key()," value = ",p.data().get()
    print ecto.dump_plasm_params(plasm)
    data = """Generate_0: {start: 3.0, step: 25.0}
Multiply_1: {factor: 1.0}
Multiply_4: {factor: 10.0}"""
    ecto.load_plasm_params(plasm, data)
    ecto.config_plasm(plasm)
    
    print ecto.dump_plasm_params(plasm)
    #ecto.view_plasm(plasm)
    
if __name__ == '__main__':
    test_iterplasm()