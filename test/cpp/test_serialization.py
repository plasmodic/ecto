#!/usr/bin/env python

import ecto
import ecto_test

def test_serialization():
    plasm = ecto.Plasm()
    gen = ecto_test.Generate("Gen", step=1.0, start=0.0)
    inc = ecto_test.Increment("Increment 0", delay=2)
    plasm.connect(gen, "out", inc, "in")
    for j in range(5): # one has already been added
        inc_next = ecto_test.Increment("Increment_%u" % (j+1), delay=2)
        plasm.connect(inc, "out", inc_next, "in")
        inc = inc_next

    printer = ecto_test.Printer("Printy")
    plasm.connect(inc, "out", printer, "in")
    plasm.save('python_graph.ecto')
    plasm.execute(4);
   
if __name__ == '__main__':
    test_serialization()

