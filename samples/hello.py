#!/usr/bin/env python
import ecto #ecto core library
import hello_ecto #a user library, that has a few ecto modules

debug = True

def mygraph():
    #instantiate a plasm, our DAG structure
    plasm = ecto.Plasm()
    
    #instantiate processing modules
    r = hello_ecto.Reader()
    
    
    
    #notice the keyword args, these get mapped
    #as parameters
    p1 = hello_ecto.Printer(str="default")
    p2 = hello_ecto.Printer(str="default")
    val = r.outputs["output"]
    print val
    
    delay = hello_ecto.Delay(type_of=r.outputs["output"], default_value = "()")
    
    (sink,source) = make_teleport(r.output["output"])
    
    #connect outputs to inputs
    plasm.connect(
                  r["output"] >> delay["input"],
                  delay["output"] >> p2["str"]
                  )
    
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
