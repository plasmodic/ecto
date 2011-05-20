#!/usr/bin/env python
import ecto
import hello_ecto

def mygraph():
    plasm = ecto.Plasm()
    help(hello_ecto)
    r = hello_ecto.Reader()
    p1 = hello_ecto.Printer(str="default")
    p2 = hello_ecto.Printer(str="default")
    plasm.connect(r, "output", p1, "str")
    plasm.connect(r, "output", p2, "str")
    print plasm.viz()
    ecto.view_plasm(plasm)
    print "Enter input, q to quit"
    while r.outputs.output != 'q':
        plasm.execute()

if __name__ == '__main__':
    mygraph()
