#!/usr/bin/env python

import ecto
import ecto_test

def test_feedback():
    plasm = ecto.Plasm()
    g = ecto_test.Generate("Generator", step=1.0, start=1.0)
    add = ecto_test.Add()
    source,sink = ecto.EntangledPair()
    plasm.connect(source[:] >> add['left'],
                  g[:] >> add['right'],
                  add[:] >> sink[:]
                  )
    #ecto.view_plasm(plasm)
    plasm.execute(niter=1)
    assert add.outputs.out == 1 # 0 + 1 = 1
    plasm.execute(niter=1)
    assert add.outputs.out == 3 # 1 + 2 = 3
    plasm.execute(niter=1)
    assert add.outputs.out == 6 # 3 + 3 = 6
    plasm.execute(niter=1)
    assert add.outputs.out == 10 # 6 + 4 = 10
if __name__ == '__main__':
    test_feedback()


