#!/usr/bin/env python

import ecto
import ecto_test

def test_feedback():
    plasm = ecto.Plasm()
    g = ecto_test.Generate("Generator", step=1.0, start=1.0)
    add = ecto_test.Add()

    printy = ecto_test.Printer()

    delay = ecto.Delay(default=0.0, qsize=1)
    
    plasm.connect(delay.source[:] >> (add['left'], printy[:]),
                  g[:] >> add['right'],
                  add[:] >> delay.sink[:]
                  )
    # ecto.view_plasm(plasm)
    plasm.execute(niter=1)
    print add.outputs.out
    assert add.outputs.out == 1 # 0 + 1 = 1
    plasm.execute(niter=1)
    print add.outputs.out
    assert add.outputs.out == 3 # 1 + 2 = 3
    plasm.execute(niter=1)
    print add.outputs.out
    assert add.outputs.out == 6 # 3 + 3 = 6
    plasm.execute(niter=1)
    print add.outputs.out
    assert add.outputs.out == 10 # 6 + 4 = 10

if __name__ == '__main__':
    test_feedback()


