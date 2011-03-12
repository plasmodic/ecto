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





