#!/usr/bin/env python
import ecto
import hello_ecto

def declare_graph():
    plasm = ecto.Plasm()
    help(hello_ecto)
    r = hello_ecto.Reader()
    p = hello_ecto.Printer(str="default")
    plasm.connect(r, "output", p, "str")
    print plasm.viz()
    print "Enter input, q to quit"
    while r.outputs.output != 'q':
        plasm.execute()

if __name__ == '__main__':
    declare_graph()
