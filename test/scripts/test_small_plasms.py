#!/usr/bin/env python

import ecto
import ecto_test

gen = ecto_test.Generate(start=0, step=1)
plasm = ecto.Plasm()
plasm.insert(gen)
plasm.execute(niter=1)
result = gen.outputs.out
print result
assert(result == 0)

plasm.execute(niter=1)
result = gen.outputs.out
print result
assert(result == 1)

plasm.execute(niter=1)
result = gen.outputs.out
print result
assert(result == 2)

