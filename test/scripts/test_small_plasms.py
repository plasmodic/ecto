#!/usr/bin/env python

import ecto
import ecto_test

def test_onenode():
    gen = ecto_test.Generate(start=0, step=1)
    plasm = ecto.Plasm()
    plasm.insert(gen)
    print ">>>", ecto
    print ">>>", ecto.schedulers
    print ">>>", ecto.schedulers.Singlethreaded
    scheduler = ecto.schedulers.Singlethreaded(plasm)
    print scheduler
    scheduler.execute()
    result = gen.outputs.out
    print result
    assert(result == 0)

    scheduler.execute()
    result = gen.outputs.out
    print result
    assert(result == 1)

    scheduler.execute()
    result = gen.outputs.out
    print result
    assert(result == 2)

if __name__ == '__main__':
    test_onenode()



