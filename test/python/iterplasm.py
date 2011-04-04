#!/usr/bin/env python

import ecto, buster, makeplasm

strs = {ecto.vertex_t.output : "OUTPUT", ecto.vertex_t.input:"INPUT",ecto.vertex_t.root:"ROOT"}
def print_vertex(vert):
    return vert[0].Name() + ":"+strs[vert[1]]+"(" + vert[2]+")"
plasm = makeplasm.makeplasm()
 
vertices = plasm.vertices()
edges = plasm.edges()

for s, t in edges:
    source = vertices[s]
    target = vertices[t]   
    print print_vertex(source), " => ", print_vertex(target)

print vertices
for k,(x,t,name,tendril) in vertices.items():
    if(t == ecto.vertex_t.root):
        print x.Name(), " has parameters:"
        for p in x.params:
            print p.key()," value = ",p.data().get()

module = plasm.toModule()
module2 = makeplasm.makeplasm().toModule()
print moduleDoc(module)
ecto.view_plasm(plasm)