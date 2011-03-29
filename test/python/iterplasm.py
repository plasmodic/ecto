#!/usr/bin/env python

import ecto, buster

plasm = ecto.Plasm()
g = buster.Generate()
g.Params(g.params)
g.params['step'].set(3.0)
g.params['start'].set(4.0)
g.Config()

m = buster.Multiply()
m.Params(m.params)
m.params['factor'].set(13.0)
m.Config()

plasm.connect(g, 'out', m, 'in')

print plasm


for pr in plasm.edges:
    n = pr.key()
    e = pr.data()
    print n, " => ", e
    print n.Name(), len(n.inputs), len(n.outputs), len(n.params)
