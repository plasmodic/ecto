#!/usr/bin/env python

import ecto
from ecto.doc import printModuleDoc
import buster

g = buster.Generate()
g.Config(17, 3)
printModuleDoc(g)

m = buster.Multiply()
m.Params(m.params)
printModuleDoc(m)

m.Config(2)
printModuleDoc(m)

fan = buster.Scatter()
fan.Config(10,10)
printModuleDoc(fan)

idx = buster.Indexer()
idx.Config(2)
printModuleDoc(idx)
print "scatter doc"
fan = buster.Scatter()
fan.Config(10,10)
gather = buster.Gather()
gather.Config(10)
printModuleDoc(gather)

g.connect("out", m, "in")

g.process()

print "gout:", g.outputs["out"].value()

m.process()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()

g.process()
print "gout:", g.outputs["out"].value()

m.process()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()

print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()
m2 = buster.Multiply()
m3 = buster.Multiply()
m2.Params(m2.params)
m2.Config(3)
m3.Params(m3.params)
m3.Config(5)
m.Config(5)
g.Config(17, 3)
try:
    plasm.connect(g, "out", m, "fin")
except RuntimeError,e:
    print "caught a type mismatch"
    print e
plasm.connect(g, "out", m, "in")
plasm.connect(m, "out", m2, "in")
plasm.connect(m, "out", m3, "in")
for x in range(0,10):
    idx = buster.Indexer()
    idx.Config(x)
    plasm.connect(fan,"out",idx,"in")
    plasm.connect(idx,"out",gather,"in_%04d"%x)
plasm.markDirty(fan)
plasm.markDirty(g)
plasm.go(m3)
plasm.go(m2)
plasm.go(gather)
print "digraph plasm {"
print plasm.viz()
print "}"
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()
print "m2out:", m2.outputs["out"].value()
print "m3out:", m3.outputs["out"].value()
print "scatter out:", fan.outputs["out"].value()
print "gather out:", gather.outputs["out"].value()




