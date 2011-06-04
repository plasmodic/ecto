#!/usr/bin/env python
import i
import ecto
import ecto_test

def test_onenode():
    gen = ecto_test.Generate(start=0, step=1)
    plasm = ecto.Plasm()
    plasm.insert(gen)
    plasm.execute()
    result = gen.outputs.out
    print result
    assert(result == 0)

    plasm.execute()
    result = gen.outputs.out
    print result
    assert(result == 1)

    plasm.execute()
    result = gen.outputs.out
    print result
    assert(result == 2)

if __name__ == '__main__':
    test_onenode()



