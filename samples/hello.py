#!/usr/bin/env python
import ecto #ecto core library
import hello_ecto #a user library, that has a few ecto modules

debug = False

def mygraph():
    #instantiate a plasm, our DAG structure
    plasm = ecto.Plasm()
    
    #instantiate processing modules
    r = hello_ecto.Reader()
    
    #notice the keyword args, these get mapped
    #as parameters
    p1 = hello_ecto.Printer(str="default")
    p2 = hello_ecto.Printer(str="default")
    
    #connect outputs to inputs
    plasm.connect(r, "output", p1, "str")
    plasm.connect(r, "output", p2, "str")
    
    if debug:
        #render the DAG with dot
        print plasm.viz()
        ecto.view_plasm(plasm)
    
    #an execution loop
    print "Enter input, q to quit"
    while r.outputs.output != 'q':
        plasm.execute() #this executes the graph in compiled code.

if __name__ == '__main__':
    mygraph()