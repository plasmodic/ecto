#!/usr/bin/env python
import ecto, ecto.schedulers
import ecto_test

class BlackBox():
    
        
def make_plasm(plasm):
    generate = ecto_test.Generate(start=1, step=3.0)
    inc = ecto_test.Increment()
    plasm.connect(generate, "out", inc, "in")
    plasm.inc = inc
    return plasm

def test_blackbox():

    
if __name__ == '__main__':
    test_blackbox()
