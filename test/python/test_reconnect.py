'''
Created on Apr 8, 2011

@author: erublee
'''
import ecto,buster


def test_reconnect():
    plasm = ecto.Plasm()
    g = ecto.make(buster.Generate,start=2, step=2)    
    m = ecto.make(buster.Multiply, factor=2)
    m2 = ecto.make(buster.Multiply, factor=2)
        
    gather = ecto.make(buster.Gather_double, n=2)
    ecto.print_module_doc(gather)
    plasm.connect(g, "out", m , "in")
    plasm.connect(g, "out", m2 , "in")
    try:
        plasm.connect(m2,"out",m,"in")
        assert(False)
    except Exception,e:
        print "Reconnect caught: ",e
    plasm.connect(m2, "out", gather , "in_0000")
    plasm.connect(m, "out", gather , "in_0001")
    try:
        plasm.connect(m2, "out", gather , "in_0001")
        assert(False)
    except Exception,e:
        print "Reconnect caught: ",e
    
    #check some values
    plasm.go(gather)
    print "Gather out :", gather.o.out.val;
    assert(gather.o.out.val == 2 *(2*2))
    plasm.go(gather)
    #should remain unchanged
    assert(gather.o.out.val == 2*(2*2))
    plasm.mark_dirty(g) #mark top of tree dirty (propagates down)
    plasm.go(gather)
    print "Gather out :", gather.o.out.val;
    assert(g.o.out.val == 4)
    assert(gather.o.out.val == 2*(2*4)) #g should be at 4 here
    #ecto.view_plasm(plasm)

if __name__ == "__main__":
    test_reconnect()