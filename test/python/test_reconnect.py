'''
Created on Apr 8, 2011

@author: erublee
'''
import ecto,ecto_test

def test_one_to_many():
    plasm = ecto.Plasm()
    g = ecto_test.Generate(start=2, step=2)
    modules = []
    for x in range(0,5):
        m = ecto_test.Multiply(factor=2)
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
    g = ecto_test.Generate(start=2, step=2)    
    m = ecto_test.Multiply(factor=2)
    m2 = ecto_test.Multiply(factor=2)
    gather = ecto_test.Gather_double(n=2)
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
    plasm.execute()
    print gather.outputs.out
    assert(gather.outputs.out == 2 *(2*2))
    
if __name__ == "__main__":
    test_reconnect()
    test_one_to_many()
