#!/usr/bin/env python
import ecto
import ecto_test
import sys


def test_required_param():
    plasm = ecto.Plasm()
    print ecto_test.RequiredInput.__doc__
    #test
    assert "REQUIRED" in ecto_test.RequiredInput.__doc__
    #test doc default value printing printing
        
    req = ecto_test.RequiredInput("Required")
    gen = ecto_test.Generate("Generator")
    printer = ecto_test.Printer("Printy")
    plasm.connect(gen[:] >> req[:],
                   req[:] >> printer[:])
    plasm.execute(3)
    print req.outputs.out

if __name__ == '__main__':
    test_required_param()
    
