#!/usr/bin/env python
import ecto
import ecto_test

def test_plasm():
    scatter = ecto_test.Scatter(n=3, x=3)
    scatter2 = ecto_test.Scatter(n=5, x=10)

    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=5)
    plasm = ecto.Plasm()
    
    plasm.connect(
                  scatter[:] >> gather[:],
                  scatter2[:] >> gather2[:]
                  )
    
    plasm.execute()
    
    #tests introspection
    viz = plasm.viz()
    print viz
    assert(type(viz) == str)
    
    result1 = gather.outputs.out
    print result1
    assert(result1 == 9) # 3 * 3
    result2 = gather2.outputs.out
    print result2
    assert(result2 == 50) # 5 * 10

def bad_syntax_errors():
    scatter = ecto_test.Scatter(n=3, x=3)
    scatter2 = ecto_test.Scatter(n=5, x=10)

    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=5)

    try:
        plasm = ecto.Plasm()
        plasm.connect(
                  scatter[:] >> gather2[:]
                  )
        assert False, "Should not work as there is a size mismatch..."
    except Exception, e:
        print e

if __name__ == '__main__':
    test_plasm()
    bad_syntax_errors()



