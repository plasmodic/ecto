#!/usr/bin/env python

import ecto


m = ecto.OurModule()

i = m.inputs['in']
print i, i.type_name()

o1 = m.outputs['out1']
print o1, o1.type_name()

o2 = m.outputs['out2']
print o2, o2.type_name()





g = ecto.Generate();
g.Config(17, 3)

m = ecto.Multiply();
m.Config(2);

g.connect("out", m, "in")

g.Process()

print "gout:", g.outputs["out"].value()

m.Process()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()

g.Process()
print "gout:", g.outputs["out"].value()

m.Process()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()

print "#################\nPlasm test\n#################"
plasm = ecto.Plasm()
m2 = ecto.Multiply()
m3 = ecto.Multiply()
m4 = ecto.Multiply()
m4.Config(4)
m2.Config(3)
m3.Config(5)
m.Config(5)
g.Config(17, 3)
plasm.connect(g, "out", m, "in")
plasm.connect(g, "out", m2, "in")
plasm.connect(m2, "out", m3, "in")
plasm.connect(m2, "out", m4, "in")
plasm.markDirty(g)
plasm.go(m)
plasm.go(m4)
print plasm.viz()
print "gout:", g.outputs["out"].value()
print "mout:", m.outputs["out"].value()
print "m2out:", m2.outputs["out"].value()
print "m3out:", m3.outputs["out"].value()




