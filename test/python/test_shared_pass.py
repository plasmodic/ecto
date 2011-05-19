

#!/usr/bin/env python
import ecto
#from ecto.doc import print_module_doc
import buster

def test_plasm():
    plasm = ecto.Plasm()
    sps = []
    for i in range(0,7):
        sps.append(buster.SharedPass(x=i));
        
    plasm.connect(sps[6],"output",sps[1],"input");
    plasm.connect(sps[1],"output",sps[2],"input");
    plasm.connect(sps[6],"output",sps[0],"input");
    plasm.connect(sps[0],"output",sps[4],"input");
    plasm.connect(sps[4],"output",sps[5],"input");

    plasm.execute()
    #ecto.view_plasm(plasm)

    assert sps[2].outputs.value == 6
    assert sps[4].outputs.value == 6
    assert sps[6].outputs.value == 6
if __name__ == '__main__':
    test_plasm()



