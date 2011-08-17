#!/usr/bin/env python
import ecto
import ecto_test
import sys


def test_required_param():
    plasm = ecto.Plasm()
    print ecto_test.RequiredParam.__doc__
    #test
    assert "REQUIRED" in ecto_test.RequiredParam.__doc__
    #test doc default value printing printing
    assert "2.1253" in ecto_test.RequiredParam.__doc__
    try:
        req = ecto_test.RequiredParam("Required")
        assert False, "should have thrown"
    except RuntimeError, e:
        print e
        
    req = ecto_test.RequiredParam("Required", x=2.2)
    assert req.params.at("x").required == True

    gen = ecto_test.Generate("Generator")
    printer = ecto_test.Printer("Printy")
    plasm.connect(gen[:] >> req[:],
                   req[:] >> printer[:])
    plasm.execute(niter=3)
    assert req.outputs.out == 6.2

if __name__ == '__main__':
    test_required_param()




