#!/usr/bin/env python

import ecto
import ecto_test

def test_feedback():
    plasm = ecto.Plasm()
    g = ecto_test.Generate("Generator", step=1.0, start=1.0)
    add = ecto_test.Add()
    print ecto.EntangledPair.__doc__
    source,sink = ecto.EntangledPair(value=add.inputs.at('left'))
    plasm.connect(source[:] >> add['left'],
                  g[:] >> add['right'],
                  add[:] >> sink[:]
                  )
    previous = 0
    for i in range(1,5):
        plasm.execute(niter=1)
        print add.outputs.out
        print "expected: ",i + previous
        assert add.outputs.out == i + previous# 0 + 1 = 1
        previous = i + previous

if __name__ == '__main__':
    test_feedback()


