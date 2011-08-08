#!/usr/bin/env python

import ecto
import ecto_test

def test_constant2():
    print "test running.."
    plasm = ecto.Plasm()
    c = ecto.Constant(value=0.50505)
    m = ecto_test.Multiply(factor=3.3335)
    passthrough = ecto.Passthrough()
    print passthrough.__doc__
    print ">>>>DOC>>>>", c.__doc__
    pr = ecto_test.Printer()

    p = ecto.Plasm()

    plasm.connect(c[:] >> m[:],
                  m[:] >> passthrough[:],
                  passthrough[:] >> pr[:]
                  )

    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)
    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)
    
def test_constant():
    print "test running.."
    plasm = ecto.Plasm()
    c = ecto.Constant(value=0.50505)
    m = ecto_test.Multiply(factor=3.3335)
    passthrough = ecto_test.PassthroughAny()
    print passthrough.__doc__
    print ">>>>DOC>>>>", c.__doc__
    pr = ecto_test.Printer()

    p = ecto.Plasm()

    plasm.connect(c[:] >> m[:],
                  m[:] >> passthrough[:],
                  passthrough[:] >> pr[:]
                  )

    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)
    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)

if __name__ == '__main__':
    test_constant()
    test_constant2()


