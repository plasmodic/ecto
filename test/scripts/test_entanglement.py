#!/usr/bin/env python

import ecto
import ecto_test

def test_feedback():
    plasm = ecto.Plasm()
    g = ecto_test.Generate("Generator", step=1.0, start=1.0)
    add = ecto_test.Add()
<<<<<<< HEAD

    printy = ecto_test.Printer()

    delay = ecto.Delay(default=0.0, qsize=1)
    
    plasm.connect(delay.source[:] >> (add['left'], printy[:]),
=======
    print ecto.EntangledPair.__doc__
    source,sink = ecto.EntangledPair(value=add.inputs.at('left'))
    plasm.connect(source[:] >> add['left'],
>>>>>>> 26832028659a8588fd8c32d2a4418a5ab2e9d7a6
                  g[:] >> add['right'],
                  add[:] >> delay.sink[:]
                  )
<<<<<<< HEAD
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

=======
    previous = 0
    for i in range(1,5):
        plasm.execute(niter=1)
        print add.outputs.out
        print "expected: ",i + previous
        assert add.outputs.out == i + previous# 0 + 1 = 1
        previous = i + previous
>>>>>>> 26832028659a8588fd8c32d2a4418a5ab2e9d7a6
if __name__ == '__main__':
    test_feedback()


