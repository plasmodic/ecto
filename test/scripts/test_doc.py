#!/usr/bin/env python
import ecto
import ecto_test
import pyecto

def test_doc():
    scatter = ecto_test.Scatter(n=6, x=3)
    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=3)
    plasm = ecto.Plasm()
    plasm.connect(
                  scatter["out_0000","out_0001","out_0002"] >> gather[:],
                  scatter["out_0003","out_0004","out_0005"] >> gather2[:]
                  )
    plasm.execute()
    result = gather.outputs.out
    assert(result == 9) # 3 * 3
    assert scatter.__doc__ != None
    print scatter.__doc__
    print gather.__doc__
    print plasm.viz()

def test_inspection():
    ecto.list_ecto_module(ecto_test)
    ecto.list_ecto_module(pyecto)

if __name__ == '__main__':
    test_doc()
    test_inspection()
