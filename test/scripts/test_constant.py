#!/usr/bin/env python

import ecto
import ecto_test

def test_constant():
    plasm = ecto.Plasm()
    c = ecto.Constant(value=0.50505)
    pr = ecto_test.Printer()

    p = ecto.Plasm()

    plasm.connect(c[:] >> pr[:])


    plasm.execute()


if __name__ == '__main__':
    test_constant()



