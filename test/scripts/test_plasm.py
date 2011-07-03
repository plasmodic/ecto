#!/usr/bin/env python
import ecto
import ecto_test

def test_plasm():
    scatter = ecto_test.Scatter(n=3, x=3)
    scatter2 = ecto_test.Scatter(n=5, x=10)

    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=5)
    plasm = ecto.Plasm()

    p = ecto.Plasm()
    #test the old syntax.
    p.connect(scatter, "out_0000", gather, "in_0000")

    try:
        p2 = ecto.Plasm()
        p2.connect(gather["out"] >> ecto_test.Printer(print_type="double")["in"])
        assert False, "Should not work as there is a type mismatch..."
    except RuntimeError, e:
        print ">>>", e
        assert str(e) == "type mismatch:  'ecto_test::Gather<int>.outputs.out' of type 'int' is connected to'ecto_test::Printer.inputs.in' of type 'double'"
        print e, "(threw as expected)"
    
    try:
        p2 = ecto.Plasm()
        p2.connect(gather["out"],ecto_test.Printer(print_type="double")["in"])
        assert False, "Should not work."
    except RuntimeError, e:
        print e
        assert 'Did you mean' in str(e)
        
    plasm.connect(scatter[:] >> gather[:],
                  scatter2[:] >> gather2[:],
                  gather["out"] >> ecto_test.Printer(print_type="int")["in"]
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

    l = plasm.connections()
    print l

    plasm2 = ecto.Plasm()
    plasm2.connect(l)
    assert plasm.viz() == plasm2.viz()
    assert l == plasm2.connections()
    plasm2.execute()
    assert gather2.outputs.out == 50

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
    except RuntimeError, e:
        print e

if __name__ == '__main__':
    test_plasm()
    bad_syntax_errors()



