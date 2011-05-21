'''
Created on Apr 8, 2011

@author: erublee
'''
import ecto,buster

def test_one_to_many():
    plasm = ecto.Plasm()
    g = buster.Generate(start=2, step=2)
    modules = []
    for x in range(0,5):
        m = buster.Multiply(factor=2)
        plasm.connect(g,"out",m,"in")
        modules.append(m)
        
    plasm.execute()
    for x in modules:
        #print x.outputs.out
        assert(x.outputs.out == 4)
    plasm.execute()
    for x in modules:
        #print x.outputs.out
        assert(x.outputs.out == 8)
    

def test_reconnect():
    plasm = ecto.Plasm()
    g = buster.Generate(start=2, step=2)    
    m = buster.Multiply(factor=2)
    m2 = buster.Multiply(factor=2)
    gather = buster.Gather_double(n=2)
    plasm.connect(g, "out", m , "in")
    plasm.connect(g, "out", m2 , "in")
    try:
        plasm.connect(m2,"out",m,"in")
        assert(False) #reconnection bad...
    except RuntimeError,e:
        pass
        #print "Reconnect caught: ",e
    plasm.connect(m2, "out", gather , "in_0000")
    plasm.connect(m, "out", gather , "in_0001")
    try:
        plasm.connect(m2, "out", gather , "in_0001")
        assert(False)
    except RuntimeError,e:
        pass
    plasm.disconnect(m, "out", gather , "in_0001")
    plasm.connect(m, "out", gather , "in_0001")


    
    #ecto.view_plasm(plasm)
    #check some values
    plasm.go(gather)
    print gather.outputs.out
    assert(gather.outputs.out == 2 *(2*2))
    plasm.go(gather)
    #should remain unchanged
    assert(gather.outputs["out"].val == 2*(2*2))
    plasm.mark_dirty(g) #mark top of tree dirty (propagates down)
    plasm.go(gather)
    assert(g.outputs["out"].val == 4)
    assert(gather.outputs["out"].val == 2*(2*4)) #g should be at 4 here
    #ecto.view_plasm(plasm)

if __name__ == "__main__":
    test_reconnect()
    test_one_to_many()
