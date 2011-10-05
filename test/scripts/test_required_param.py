#!/usr/bin/env python
import ecto, ecto_test, sys, util


def test_required_param():
    plasm = ecto.Plasm()
    print "<DOC>", ecto_test.RequiredParam.__doc__, "</DOC>"
    #test
    assert "REQUIRED" in ecto_test.RequiredParam.__doc__
    #test doc default value printing printing
    assert "2.1253" in ecto_test.RequiredParam.__doc__
    try:
        req = ecto_test.RequiredParam("Required")
        print "egh, didn't throw"
        util.fail()
    except RuntimeError, e:
        print "Yup, there is our throw:", e
        
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




