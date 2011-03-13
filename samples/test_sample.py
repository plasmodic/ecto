#!/bin/python
import ecto
from ecto.doc import printModuleDoc
import sample

plasm = ecto.Plasm()

g = sample.Generate()
g.Config(17, 3)
printModuleDoc(g)

m = sample.Multiply()
m.Config(2)
printModuleDoc(m)

plasm.connect(g, "out", m , "in")

for i in range(0,10):
  plasm.markDirty(g)
  plasm.go(m)
  print m.outputs["out"].value()
