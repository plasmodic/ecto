#!/usr/bin/env python
import ecto
import ecto_test

def test_plasm():
    plasm = ecto.Plasm()
    sps = []
    for i in range(0,7):
        sps.append(ecto_test.SharedPass(x=i));
        
    plasm.connect(sps[6],"output",sps[1],"input");
    plasm.connect(sps[1],"output",sps[2],"input");
    plasm.connect(sps[6],"output",sps[0],"input");
    plasm.connect(sps[0],"output",sps[4],"input");
    plasm.connect(sps[4],"output",sps[5],"input");

    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=1)
    #ecto.view_plasm(plasm)

    print sps[2].outputs.value
    assert sps[2].outputs.value == 6
    assert sps[4].outputs.value == 6
    assert sps[6].outputs.value == 6
if __name__ == '__main__':
    test_plasm()



