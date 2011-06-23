#!/usr/bin/env python

import ecto
import ecto_test

def test_constant():
    print "test running.."
    plasm = ecto.Plasm()
    c = ecto.Constant(value=0.50505)
    pr = ecto_test.Printer(print_type="double")

    p = ecto.Plasm()

    plasm.connect(c[:] >> pr[:])

    print plasm.viz()
    plasm.execute()

    assert c.outputs.out == 0.50505
    plasm.execute()

    assert c.outputs.out == 0.50505

if __name__ == '__main__':
    test_constant()



