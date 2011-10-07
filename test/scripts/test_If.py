#!/usr/bin/env python

import ecto
import ecto_test

def test_If():
    plasm = ecto.Plasm()
    g = ecto_test.Generate("Generator", step=1.0, start=1.0)
    If = ecto.If(cell=g)
    truer = ecto.TrueEveryN(n=3,count=3)
    plasm.connect(truer['flag'] >> If['__test__']
                  )
    plasm.execute(niter=27)
    assert g.outputs.out == 9 #should have only called execute 9 times.
if __name__ == '__main__':
    test_If()


