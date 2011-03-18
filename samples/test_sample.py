#!/bin/python
import ecto
from ecto.doc import printModuleDoc
import sample

plasm = ecto.Plasm()

g = sample.Generate()
g.Params(g.params)
g.Config()
printModuleDoc(g)

m = sample.Multiply()
m.Params(m.params)
m.Config()
printModuleDoc(m)

plasm.connect(g, "out", m , "in")

for i in range(0,10):
  plasm.markDirty(g)
  plasm.go(m)
  print m.outputs["out"].get()
