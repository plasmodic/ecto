#!/bin/python
import ecto
from ecto.doc import printModuleDoc
import buster

plasm = ecto.Plasm()

g = buster.Generate()
g.Params(g.params)
g.Config()
printModuleDoc(g)

m = buster.Multiply()
m.Params(m.params)
m.Config()
printModuleDoc(m)

plasm.connect(g, "out", m , "in")

print plasm.viz()
for i in range(0,10):
  plasm.markDirty(g)
  plasm.go(m)
  print "output: ", m.outputs["out"].get()

