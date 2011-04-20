#!/usr/bin/env python
import ecto
import buster

def test_tendril():
    tendril = ecto.Tendril()
    tendril.set(5)
    t = ecto.Tendril()
    t.val = 5
    assert t.val == t.get()
    assert t.val == 5
    assert tendril.val == t.val
    assert tendril.get() == 5
    assert tendril.get() == t.get()
    tendril.val = 10
    t.val = tendril.val
    t.val = "hi"
    assert tendril.val != "hi"
    assert t.val == t.get()
    assert t.val == "hi"
if __name__ == '__main__':
    test_tendril()
